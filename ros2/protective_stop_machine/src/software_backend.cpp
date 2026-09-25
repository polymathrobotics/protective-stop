// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0
//
// SoftwareMachineBackend — hosts the certified pstop_c machine in-process on a
// dedicated thread, binding UDP so remotes bond directly. Mirrors the transport
// + poll loop of host/machine_app_runner.c; pstop_c is linked unchanged. The
// machine's authoritative state is read from machine.robot_state each cycle.
#include "protective_stop_machine/software_backend.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

extern "C"
{
#include "pstop/machine.h"
#include "pstop/pstop_application.h"
#include "pstop/pstop_msg.h"
#include "pstop/pstop_remote_data.h"
#include "pstop_aux_channel.h"
#include "transport/udp/udp_transport.h"
}

namespace protective_stop_machine
{

struct SoftwareMachineBackend::Impl
{
  SoftwareConfig config;

  pstop_application_t application{};
  pstop_machine_t machine{};
  udp_transport_data_t udp_transport{};
  std::vector<pstop_remote_data_t> clients;

  std::thread thread;
  std::atomic<bool> running{false};
  std::atomic<bool> reachable{false};

  mutable std::mutex state_mutex;
  // guarded by state_mutex
  MachineSnapshot latest_snapshot;
  // guarded by state_mutex; read by the C remote-details callback
  MachineTiming timing;

  // The callback has no context argument. These fields expose the announced
  // role of the frame currently being processed (machine thread only); the
  // callback seeds a NEW client's is_stop_only from it at BOND.
  uint32_t frame_role_id{0};
  pstop_aux_role_t frame_role{PSTOP_AUX_ROLE_UNSPECIFIED};

  void run();
  // caller holds no lock; locks internally
  void rebuild_snapshot();
};

// The pstop_c callbacks are plain C function pointers with no user context.
// There is at most one software backend, so bridge them through a file-scope
// pointer set for the lifetime of the running loop.
static SoftwareMachineBackend::Impl * g_active_impl = nullptr;

static uint64_t now_ms()
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<uint64_t>(now.tv_sec) * 1000ULL + static_cast<uint64_t>(now.tv_nsec) /
         1000000ULL;
}

static remote_details_t resolve_remote_details(const device_id_t * device_id)
{
  remote_details_t details;
  remote_detail_init(&details);
  uint64_t heartbeat_ms = 400;
  bool allow = true;
  bool stop_only = true;
  if (g_active_impl) {
    const uint32_t remote_id = (device_id != nullptr) ? device_id->data : 0U;
    {
      std::lock_guard<std::mutex> lock(g_active_impl->state_mutex);
      heartbeat_ms = g_active_impl->timing.heartbeat_ms;
      // ADMISSION: optional allow/denylist; both empty => admitted. A refused
      // id gets an UNBOND reply (pstop_c prepares it; run() sends it).
      allow = (remote_id != 0U) && software_remote_admitted(g_active_impl->config, remote_id);
    }
    // AUTHORITY: the remote alone declares stop-only vs operator in every
    // frame. Seeded here at BOND from the staged frame's role; refreshed per
    // frame in run() so a live role change follows without a re-bond.
    // Unspecified (old firmware / bad decode) is stop-only = fail-safe.
    const pstop_aux_role_t role =
      (g_active_impl->frame_role_id == remote_id) ? g_active_impl->frame_role :
      PSTOP_AUX_ROLE_UNSPECIFIED;
    stop_only = !pstop_aux_role_is_operator(role);
  }
  remote_detail_set(&details, allow, heartbeat_ms, stop_only);
  return details;
}

static void ignore_status(pstop_status_message_t /*status*/)
{
  // State is derived authoritatively from machine.robot_state in the loop; the
  // callback is intentionally a no-op (avoids the early-OK latch subtlety).
}

static void ignore_log(uint64_t, const device_id_t *, uint8_t, pstop_error_t)
{}

SoftwareMachineBackend::SoftwareMachineBackend(const SoftwareConfig & config)
: impl_(std::make_unique<Impl>())
{
  impl_->config = config;
  impl_->timing = config.timing;
}

SoftwareMachineBackend::~SoftwareMachineBackend()
{
  stop();
}

bool SoftwareMachineBackend::start()
{
  if (impl_->running.load()) {
    return true;
  }
  // The pstop_c callbacks reach the instance through the file-scope g_active_impl.
  // Refuse a second concurrent instance rather than silently clobber it.
  if (g_active_impl != nullptr && g_active_impl != impl_.get()) {
    return false;
  }
  g_active_impl = impl_.get();

  // bonded-remote slot pool
  constexpr uint16_t kMaxRemotes = 4;
  impl_->clients.assign(kMaxRemotes, pstop_remote_data_t{});
  // Seed app_config from the latest stashed timing (configure() may have run
  // while inactive); the machine thread keeps it current thereafter.
  impl_->config.timing = impl_->timing;

  transport_udp_init(&impl_->udp_transport);
  pstop_application_init(&impl_->application);
  impl_->application.app_config.max_lost_messages = 10;
  impl_->application.app_config.max_missed_heartbeats = impl_->config.timing.max_missed;
  impl_->application.app_config.delay_between_stop_ms =
    static_cast<uint32_t>(impl_->config.timing.min_stop_ms);
  impl_->application.remote_details_cb = resolve_remote_details;
  impl_->application.status_cb = ignore_status;
  impl_->application.log_message_cb = ignore_log;
  impl_->application.env.get_time_cb = now_ms;

  machine_init(&impl_->machine, &impl_->application, impl_->clients.data(), kMaxRemotes);

  if (transport_udp_listen(
      &impl_->udp_transport, impl_->config.bind_addr.c_str(), impl_->config.port) < 0)
  {
    g_active_impl = nullptr;
    return false;
  }
  device_id_t device_id = {impl_->config.machine_id};
  device_id_copy(&(impl_->machine.application->machine_device_id), &device_id);

  impl_->reachable = true;
  impl_->running = true;
  impl_->thread = std::thread([this] {impl_->run();});
  return true;
}

void SoftwareMachineBackend::stop()
{
  if (!impl_->running.exchange(false)) {
    return;
  }
  if (impl_->thread.joinable()) {
    impl_->thread.join();
  }
  transport_udp_close(&impl_->udp_transport);
  impl_->reachable = false;
  {
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    // not reachable -> UNSTABLE by state()
    impl_->latest_snapshot = MachineSnapshot{};
  }
  if (g_active_impl == impl_.get()) {
    g_active_impl = nullptr;
  }
}

void SoftwareMachineBackend::Impl::run()
{
  uint8_t request_bytes[PSTOP_MESSAGE_SIZE];
  uint8_t response_bytes[PSTOP_MESSAGE_SIZE];
  pstop_msg_t req_msg;
  pstop_msg_t resp_msg;

  while (running.load()) {
    // Apply the latest timing HERE (machine thread) so app_config is only ever
    // written by this thread — no cross-thread race with configure(). heartbeat
    // reaches pstop_c via resolve_remote_details, which reads `timing` under state_mutex.
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      application.app_config.max_missed_heartbeats = timing.max_missed;
      application.app_config.delay_between_stop_ms = static_cast<uint32_t>(timing.min_stop_ms);
    }
    machine_validate_heartbeats(&machine);

    struct sockaddr_storage client_addr;
    int bytes_read =
      transport_udp_read(&udp_transport, request_bytes, PSTOP_MESSAGE_SIZE, &client_addr);
    if (bytes_read == PSTOP_MESSAGE_SIZE) {
      pstop_message_decode(&req_msg, request_bytes);
      // Drop a non-BOND from an unknown/timed-out remote (upstream would
      // dereference before its NULL guard); a fresh BOND re-bonds.
      bool is_bonded = pstop_remote_get(&machine.remotes, &req_msg.id) != nullptr;

      if (is_bonded || req_msg.message == PSTOP_MESSAGE_BOND) {
        frame_role_id = 0U;
        frame_role = PSTOP_AUX_ROLE_UNSPECIFIED;
        if (req_msg.checksum == req_msg.calculated_checksum &&
          req_msg.receiver_id.data == config.machine_id)
        {
          frame_role_id = req_msg.id.data;
          frame_role = pstop_aux_decode_role(&req_msg);
          // Live role (shared policy, common/pstop_aux_channel.h): refresh the
          // bonded client's is_stop_only from THIS frame and release any
          // arming-cycle ownership a stop-only remote holds.
          if (is_bonded) {
            pstop_aux_apply_role_pre(&machine, &req_msg);
          }
        }
        pstop_message_init(&resp_msg);
        const pstop_error_t process_result = machine_process_message(&machine, &req_msg, &resp_msg);
        // A stop-only remote's STOP never opens an arming cycle.
        if (is_bonded && req_msg.checksum == req_msg.calculated_checksum) {
          pstop_aux_apply_role_post(&machine, &req_msg, process_result);
        }
        if (process_result == PSTOP_OK) {
          pstop_message_encode(&resp_msg, response_bytes);
          transport_udp_write(&udp_transport, response_bytes, PSTOP_MESSAGE_SIZE,
              reinterpret_cast<struct sockaddr_in *>(&client_addr));
        } else if (process_result == PSTOP_OPERATOR_NOT_ALLOWED) {
          // Admission refused: pstop_c prepared an UNBOND reply but left the
          // addressing blank. Fill it and send it so the remote learns it was
          // refused (it parks until a manual rebond) instead of hearing silence.
          resp_msg.id.data = config.machine_id;
          resp_msg.receiver_id.data = req_msg.id.data;
          resp_msg.received_counter = req_msg.counter;
          resp_msg.received_stamp = req_msg.stamp;
          pstop_message_encode(&resp_msg, response_bytes);
          transport_udp_write(&udp_transport, response_bytes, PSTOP_MESSAGE_SIZE,
              reinterpret_cast<struct sockaddr_in *>(&client_addr));
        }
        frame_role_id = 0U;
        frame_role = PSTOP_AUX_ROLE_UNSPECIFIED;
      }
    }
    rebuild_snapshot();
  }
}

void SoftwareMachineBackend::Impl::rebuild_snapshot()
{
  MachineSnapshot rebuilt;
  rebuilt.reachable = true;
  const robot_state_t * robot_state = machine_get_robot_state(&machine);
  rebuilt.running = robot_state->robot_state == ROBOT_STATE_OK;
  rebuilt.need_stop = robot_state->restart_state == ROBOT_RESTART_STATE_NEED_STOP;
  rebuilt.active_remotes = pstop_remote_num_active(&machine.remotes);
  // the software backend has no physical relays
  rebuilt.relay.applicable = false;
  rebuilt.relay.run = rebuilt.running;
  rebuilt.relay.relay_stop = !rebuilt.running;
  rebuilt.status_reason =
    rebuilt.running ? "armed (cleared to run)" :
    (rebuilt.need_stop ? "need_stop (awaiting arming gesture)" : "stopped");

  for (uint16_t slot = 0; slot < machine.remotes.max_remotes; ++slot) {
    const pstop_remote_data_t * remote_slot = &machine.remotes.remotes[slot];
    // A slot is a LIVE bond only if pstop_c hasn't marked it inactive.
    // pstop_c leaves remote_id/local_remote_id intact when a remote times out
    // or unbonds (it only sets remote_state = UNKNOWN, like the library's own
    // is-free tests), so filtering on the ids alone reports ghost remotes.
    // Also skip a never-populated slot (remote_id 0).
    if (remote_slot->remote_state == PSTOP_REMOTE_UNKNOWN ||
      remote_slot->remote_data.remote_id.data == 0U)
    {
      continue;
    }
    RemoteInfo remote;
    char id_text[16];
    std::snprintf(id_text, sizeof(id_text), "%08x", remote_slot->remote_data.remote_id.data);
    remote.device_id = id_text;
    // Map pstop remote_state -> bond_state (1 connecting, 2 bonded, 3 stopped).
    remote.bond_state = remote_slot->remote_state ==
      PSTOP_REMOTE_INITING ? 1 : (remote_slot->remote_state == PSTOP_REMOTE_STOPPED ? 3 : 2);
    remote.in_use = robot_state->remote_stop_id == remote_slot->local_remote_id;
    remote.stop_only = remote_slot->is_stop_only;
    // reply_age / rtt / rebonds are microlink-side metrics not tracked by
    // pstop_c; left 0 on the software backend.
    rebuilt.remotes.push_back(std::move(remote));
  }

  std::lock_guard<std::mutex> lock(state_mutex);
  latest_snapshot = std::move(rebuilt);
}

MachineSnapshot SoftwareMachineBackend::snapshot() const
{
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  return impl_->latest_snapshot;
}

bool SoftwareMachineBackend::configure(const MachineTiming & timing, std::string & error)
{
  // Just stash the (node-validated) timing. The machine thread applies it to
  // app_config each cycle (see run()), so this succeeds whether the backend is
  // running or held inactive — an operator can pre-tighten before activating.
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  impl_->timing = timing;
  // so a later start() seeds from the latest
  impl_->config.timing = timing;
  error.clear();
  return true;
}

}  // namespace protective_stop_machine
