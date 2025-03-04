import {
  ChannelId,
  FoxgloveClient,
  FoxgloveServer,
  Service,
  ServiceCallPayload,
} from "@foxglove/ws-protocol";
import { WebSocket } from "ws";
import { test, expect, suite, afterEach } from "vitest";
import signal from "./signal";
import {
  buildPstopMessage,
  ProtectiveStopMsg,
  PStopMessageReader,
  PStopMessageWriter,
  PStopServiceRequestWriter,
  PStopServiceResponseReader,
} from "./messages";
import {
  ConnectionStatus,
  ProtectiveStopUIState,
  PStopMessage,
  PStopState,
} from "./types";
import {
  MessageWriter as Ros2MessageWriter,
  MessageReader,
} from "@foxglove/rosmsg2-serialization";
import fs from "fs";
import { PStopClient } from "./client";

suite("client", () => {
  afterEach(() => {
    console.log("after each");
  });

  test("Can activate the protective stop", async () => {
    const pStopClient = new PStopClient({
      receiverId: "1111",
    });
    const resp = await pStopClient.activate();
    expect(resp.success).toBe(true);
  });

  test("Throws if no receiver id is provided", async () => {
    const pStopClient = new PStopClient({ receiverId: "" });
    expect(pStopClient.activate()).rejects.toThrowError(
      "Unknown receiver uuid"
    );
  });

  test.skip("Throws if activated twice synchronosly", async () => {
    const senderId = "2222";
    const pStopClient = new PStopClient({ receiverId: "1111", senderId });
    const promise = pStopClient.activate();
    // Immediately abort
    pStopClient.abort();

    expect(resp.success).toBe(true);
    const pStopClient2 = new PStopClient({ receiverId: "1111", senderId });
  });

  test.skip("Throws if activated twice", async () => {
    const senderId = "2222";
    const pStopClient = new PStopClient({ receiverId: "1111", senderId });
    const resp = await pStopClient.activate();
    expect(resp.success).toBe(true);

    const pStopClient2 = new PStopClient({ receiverId: "1111", senderId });
    let caughtFlag = false;
    try {
      await pStopClient2.activate();
    } catch (e) {
      expect(e).toBeInstanceOf(Error);
      expect(e.message).toBe(
        "Failed to activate: Already activated. Current state of remote is ACTIVE"
      );
      caughtFlag = true;
    }
    expect(caughtFlag).toBe(true);
    expect(pStopClient2._ui_state).toBe(
      ProtectiveStopUIState.FAILED_CONNECTION
    );
  });

  test("Can deactivate the protective stop", async () => {
    const pStopClient = new PStopClient({
      receiverId: "1111",
    });
    const active = await pStopClient.activate();
    expect(active.success).toBe(true);

    const inactive = await pStopClient.deactivate();
    expect(inactive.success).toBe(true);
  });

  test.skip("Can initiate a pstop request", async () => {
    const pStopClient = new PStopClient({
      receiverId: "1111",
    });

    const pressed = signal();
    pStopClient.addStatusListener((status) => {
      if (status === ProtectiveStopUIState.ACTIVE_PRESSED) {
        pressed.resolve();
      }
    });

    const active = await pStopClient.activate();
    expect(active.success).toBe(true);

    pStopClient.togglePStop();
    await Promise.race([
      pressed,
      new Promise((_, reject) => setTimeout(reject, 1000)),
    ]).catch(() => {
      throw new Error("PStop not pressed");
    });

    const recvMsg = signal();
    pStopClient.addMessageListener((msg) => {
      expect(msg).toBeDefined();
      expect(msg.pstop_pressed).toBe(true);
      recvMsg.resolve();
    });

    await Promise.race([
      recvMsg,
      new Promise((_, reject) => setTimeout(reject, 2000)),
    ]).catch(() => {
      throw new Error("Did not receive message");
    });

    const inactive = await pStopClient.deactivate();
    expect(inactive.success).toBe(true);
  });

  test.skip("Records missed heartbeats", async () => {
    const pStopClient = new PStopClient({
      receiverId: "1111",
    });
    const active = await pStopClient.activate();
    expect(active.success).toBe(true);

    const active_signal = signal();
    const unstable_signal = signal();
    pStopClient.addStatusListener((status) => {
      if (status === ProtectiveStopUIState.UNSTABLE_CLIENT_CONNECTION) {
        unstable_signal.resolve();
      }
    });

    pStopClient.addMessageListener((msg) => {
      if (msg.connection_status.status === ConnectionStatus.ACTIVE) {
        active_signal.resolve();
      }
    });

    await active_signal;

    /**
     * Leaky abstraction here. A future more sophisticated test would actually
     * throttle outgoing websocket traffic */
    pStopClient._stopHeartbeat();
    pStopClient._startHeartbeat(active.params.heartbeat_timeout * 2000);
    await unstable_signal;

    const inactive = await pStopClient.deactivate();
    expect(inactive.success).toBe(true);
  });

  test.skip("Can recover from missed heartbeats", async () => {
    const pStopClient = new PStopClient({
      receiverId: "1111",
    });
    const active = await pStopClient.activate();
    expect(active.success).toBe(true);

    const active_signal = signal();
    const unstable_signal = signal();
    pStopClient.addStatusListener((status) => {
      if (status === ProtectiveStopUIState.UNSTABLE_CLIENT_CONNECTION) {
        unstable_signal.resolve();
      }
    });

    pStopClient.addMessageListener((msg) => {
      if (msg.connection_status.status === ConnectionStatus.ACTIVE) {
        active_signal.resolve();
      }
    });

    await active_signal;

    /**
     * Leaky abstraction here. A future more sophisticated test would actually
     * throttle outgoing websocket traffic */
    pStopClient._stopHeartbeat();
    pStopClient._startHeartbeat(active.params.heartbeat_timeout * 2000);
    await unstable_signal;

    const recovered_active_signal = signal();
    pStopClient.addStatusListener((status) => {
      if (status === ProtectiveStopUIState.ACTIVE_UNPRESSED) {
        recovered_active_signal.resolve();
      }
    });

    pStopClient._stopHeartbeat();
    pStopClient._startHeartbeat(active.params.heartbeat_timeout * 1000);
    await recovered_active_signal;

    const inactive = await pStopClient.deactivate();
    expect(inactive.success).toBe(true);
  });

  test.skip("Watchdog catches unstable robot connections and can recover", async () => {
    const pStopClient = new PStopClient({
      receiverId: "1111",
    });
    const active = await pStopClient.activate();
    expect(active.success).toBe(true);

    /**
     * Another leaky abstraction. Give the watchdog an unreasonably short timeout so it triggers.
     */
    pStopClient._startWatchdog(10);
    const unstable_watchdog_signal = signal();
    pStopClient.addStatusListener((status) => {
      if (status === ProtectiveStopUIState.UNSTABLE_ROBOT_CONNECTION) {
        unstable_watchdog_signal.resolve();
      }
    });

    await unstable_watchdog_signal;

    // Revert back to original timeout
    pStopClient._startWatchdog(active.params.heartbeat_timeout * 1000);
    const stable_watchdog_signal = signal();
    pStopClient.addStatusListener((status) => {
      if (status === ProtectiveStopUIState.ACTIVE_UNPRESSED) {
        stable_watchdog_signal.resolve();
      }
    });

    await stable_watchdog_signal;

    const inactive = await pStopClient.deactivate();
    expect(inactive.success).toBe(true);
  });
});

/**
 *
 * todo:
 * - test for never getting a message (no lastMsgTimestamp)
 * - test for degrading from unstable to failed connection
 *
 * - test for websocket connection error
 */
