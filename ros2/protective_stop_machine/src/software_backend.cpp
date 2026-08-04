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
#include "transport/udp/udp_transport.h"
}

namespace protective_stop_machine
{

struct SoftwareMachineBackend::Impl
{
  SoftwareConfig cfg;

  pstop_application_t app{};
  pstop_machine_t machine{};
  udp_transport_data_t udp{};
  std::vector<pstop_remote_data_t> clients;

  std::thread th;
  std::atomic<bool> running{false};
  std::atomic<bool> reachable{false};

  mutable std::mutex mtx;
  MachineSnapshot snap;  // guarded
  MachineTiming timing;  // guarded (read by the C remote-details callback)

  void run();
  void rebuild_snapshot();  // caller holds no lock; locks internally
};

// The pstop_c callbacks are plain C function pointers with no user context.
// There is at most one software backend, so bridge them through a file-scope
// pointer set for the lifetime of the running loop.
static SoftwareMachineBackend::Impl * g_impl = nullptr;

static uint64_t now_ms()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000ULL + static_cast<uint64_t>(ts.tv_nsec) /
         1000000ULL;
}

static remote_details_t cb_remote_details(const device_id_t * id)
{
  remote_details_t d;
  remote_detail_init(&d);
  uint64_t hb = 400;
  bool allow = true;
  // STOP-ONLY unless this remote is a listed operator. Default (empty operator
  // list) => stop-only, so an unlisted remote may STOP + is heartbeat-monitored
  // but can NEVER re-arm. Replaces the previous hardcoded is_stop_only=false,
  // which accepted every remote as a full operator.
  bool stop_only = true;
  if (g_impl) {
    std::lock_guard<std::mutex> lk(g_impl->mtx);
    hb = g_impl->timing.heartbeat_ms;
    allow = g_impl->cfg.allow_unlisted;
    stop_only = software_remote_is_stop_only(g_impl->cfg, (id != nullptr) ? id->data : 0U);
  }
  remote_detail_set(&d, allow, hb, stop_only);
  return d;
}

static void cb_status(pstop_status_message_t /*status*/)
{
  // State is derived authoritatively from machine.robot_state in the loop; the
  // callback is intentionally a no-op (avoids the early-OK latch subtlety).
}

static void cb_log(uint64_t, const device_id_t *, uint8_t, pstop_error_t)
{}

SoftwareMachineBackend::SoftwareMachineBackend(const SoftwareConfig & cfg)
: impl_(std::make_unique<Impl>())
{
  impl_->cfg = cfg;
  impl_->timing = cfg.timing;
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
  // The pstop_c callbacks reach the instance through the file-scope g_impl.
  // Refuse a second concurrent instance rather than silently clobber it.
  if (g_impl != nullptr && g_impl != impl_.get()) {
    return false;
  }
  g_impl = impl_.get();

  constexpr uint16_t kMaxRemotes = 4;  // bonded-remote slot pool
  impl_->clients.assign(kMaxRemotes, pstop_remote_data_t{});
  // Seed app_config from the latest stashed timing (configure() may have run
  // while inactive); the machine thread keeps it current thereafter.
  impl_->cfg.timing = impl_->timing;

  transport_udp_init(&impl_->udp);
  pstop_application_init(&impl_->app);
  impl_->app.app_config.max_lost_messages = 10;
  impl_->app.app_config.max_missed_heartbeats = impl_->cfg.timing.max_missed;
  impl_->app.app_config.delay_between_stop_ms =
    static_cast<uint32_t>(impl_->cfg.timing.min_stop_ms);
  impl_->app.remote_details_cb = cb_remote_details;
  impl_->app.status_cb = cb_status;
  impl_->app.log_message_cb = cb_log;
  impl_->app.env.get_time_cb = now_ms;

  machine_init(&impl_->machine, &impl_->app, impl_->clients.data(), kMaxRemotes);

  if (transport_udp_listen(&impl_->udp, impl_->cfg.bind_addr.c_str(), impl_->cfg.port) < 0) {
    g_impl = nullptr;
    return false;
  }
  device_id_t id = {impl_->cfg.machine_id};
  device_id_copy(&(impl_->machine.application->machine_device_id), &id);

  impl_->reachable = true;
  impl_->running = true;
  impl_->th = std::thread([this] {impl_->run();});
  return true;
}

void SoftwareMachineBackend::stop()
{
  if (!impl_->running.exchange(false)) {
    return;
  }
  if (impl_->th.joinable()) {
    impl_->th.join();
  }
  transport_udp_close(&impl_->udp);
  impl_->reachable = false;
  {
    std::lock_guard<std::mutex> lk(impl_->mtx);
    impl_->snap = MachineSnapshot{};  // not reachable -> UNSTABLE by state()
  }
  if (g_impl == impl_.get()) {
    g_impl = nullptr;
  }
}

void SoftwareMachineBackend::Impl::run()
{
  uint8_t reqbytes[PSTOP_MESSAGE_SIZE];
  uint8_t respbytes[PSTOP_MESSAGE_SIZE];
  pstop_msg_t req_msg;
  pstop_msg_t resp_msg;

  while (running.load()) {
    // Apply the latest timing HERE (machine thread) so app_config is only ever
    // written by this thread — no cross-thread race with configure(). heartbeat
    // reaches pstop_c via cb_remote_details, which reads `timing` under mtx.
    {
      std::lock_guard<std::mutex> lk(mtx);
      app.app_config.max_missed_heartbeats = timing.max_missed;
      app.app_config.delay_between_stop_ms = static_cast<uint32_t>(timing.min_stop_ms);
    }
    machine_validate_heartbeats(&machine);

    struct sockaddr_storage client;
    int n = transport_udp_read(&udp, reqbytes, PSTOP_MESSAGE_SIZE, &client);
    if (n == PSTOP_MESSAGE_SIZE) {
      pstop_message_decode(&req_msg, reqbytes);
      // Drop a non-BOND from an unknown/timed-out remote (upstream would
      // dereference before its NULL guard); a fresh BOND re-bonds.
      bool known = pstop_remote_get(&machine.remotes, &req_msg.id) != nullptr;
      if (known || req_msg.message == PSTOP_MESSAGE_BOND) {
        if (machine_process_message(&machine, &req_msg, &resp_msg) == PSTOP_OK) {
          pstop_message_encode(&resp_msg, respbytes);
          transport_udp_write(&udp, respbytes, PSTOP_MESSAGE_SIZE,
              reinterpret_cast<struct sockaddr_in *>(&client));
        }
      }
    }
    rebuild_snapshot();
  }
}

void SoftwareMachineBackend::Impl::rebuild_snapshot()
{
  MachineSnapshot s;
  s.reachable = true;
  const robot_state_t * rs = machine_get_robot_state(&machine);
  s.running = rs->robot_state == ROBOT_STATE_OK;
  s.need_stop = rs->restart_state == ROBOT_RESTART_STATE_NEED_STOP;
  s.active_remotes = pstop_remote_num_active(&machine.remotes);
  s.relay.applicable = false;  // software backend has no physical relays
  s.relay.run = s.running;
  s.relay.relay_stop = !s.running;
  s.status_reason =
    s.running ? "armed (cleared to run)" :
    (s.need_stop ? "need_stop (awaiting arming gesture)" : "stopped");

  for (uint16_t i = 0; i < machine.remotes.max_remotes; ++i) {
    const pstop_remote_data_t * c = &machine.remotes.remotes[i];
    // A slot is a LIVE bond only if pstop_c hasn't marked it inactive.
    // pstop_c leaves remote_id/local_remote_id intact when a remote times out
    // or unbonds (it only sets remote_state = UNKNOWN, like the library's own
    // is-free tests), so filtering on the ids alone reports ghost remotes.
    // Also skip a never-populated slot (remote_id 0).
    if (c->remote_state == PSTOP_REMOTE_UNKNOWN || c->remote_data.remote_id.data == 0U) {
      continue;
    }
    RemoteInfo r;
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%08x", c->remote_data.remote_id.data);
    r.device_id = buf;
    // Map pstop remote_state -> bond_state (1 connecting, 2 bonded, 3 stopped).
    r.bond_state = c->remote_state ==
      PSTOP_REMOTE_INITING ? 1 : (c->remote_state == PSTOP_REMOTE_STOPPED ? 3 : 2);
    r.in_use = rs->remote_stop_id == c->local_remote_id;
    r.stop_only = c->is_stop_only;
    // reply_age / rtt / rebonds are microlink-side metrics not tracked by
    // pstop_c; left 0 on the software backend.
    s.remotes.push_back(std::move(r));
  }

  std::lock_guard<std::mutex> lk(mtx);
  snap = std::move(s);
}

MachineSnapshot SoftwareMachineBackend::snapshot() const
{
  std::lock_guard<std::mutex> lk(impl_->mtx);
  return impl_->snap;
}

bool SoftwareMachineBackend::configure(const MachineTiming & timing, std::string & error)
{
  // Just stash the (node-validated) timing. The machine thread applies it to
  // app_config each cycle (see run()), so this succeeds whether the backend is
  // running or held inactive — an operator can pre-tighten before activating.
  std::lock_guard<std::mutex> lk(impl_->mtx);
  impl_->timing = timing;
  impl_->cfg.timing = timing;  // so a later start() seeds from the latest
  error.clear();
  return true;
}

}  // namespace protective_stop_machine
