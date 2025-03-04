import React, { useCallback, useEffect, useRef, useState } from "react";
import { MessageWriter as Ros2MessageWriter } from "@foxglove/rosmsg2-serialization";

import {
  Channel,
  ChannelId,
  FoxgloveClient,
  MessageData,
  ServerInfo,
  Service,
  ServiceCallPayload,
  ServiceCallRequest,
  StatusLevel,
  StatusMessage,
  SubscriptionId,
} from "@foxglove/ws-protocol";

import reactLogo from "./assets/react.svg";
import viteLogo from "/vite.svg";
import "./App.css";
import { ProtectiveStopUIState, PStopMessage } from "./types";
import { buildPstopMessage } from "./messages";
import { PStopClient } from "./client";
import PStopButton, { getProtectiveStopUI } from "./PStopButton";
import { v4 as uuid } from "uuid";

import _ from "lodash";

function useThrottle(cb, delay) {
  const options = { leading: true, trailing: false }; // add custom lodash options
  const cbRef = useRef(cb);
  // use mutable ref to make useCallback/throttle not depend on `cb` dep
  useEffect(() => {
    cbRef.current = cb;
  });
  return useCallback(
    _.throttle((...args) => cbRef.current(...args), delay, options),
    [delay]
  );
}

/**
 *
 * todo:
 * -
 *
 *
 *
 * - config
 *  - websocket url
 *  - pstop topic
 *  - pstop service
 *  - receiver id
 *  - pstop version?
 *  - timeout?
 *
 */

/**
 *todo:
- need to timeout on connecting??


 - reconnection attempt
  - reconnect now button
 - reconcile status of remote pstop with pstop node. 
 - how to err if timeout is exceeded on client?
 - add settings for receiver id, sender id, etc 
 - add nice readable errrors for each state
 * 
 */

const PStop = React.memo(() => {
  const [pStopClient, setClient] = useState<PStopClient | undefined>(undefined);
  const pStopClientRef = useRef<PStopClient | undefined>(null);
  pStopClientRef.current = pStopClient;

  const [pStopState, setPStopState] = useState<ProtectiveStopUIState>(
    ProtectiveStopUIState.DISCONNECTED
  );
  // const [status, setStatus] = useState<{ [name: string]: any }>({});
  const [lastMessage, setLastMessage] = useState<{
    timestamp: number;
    msg: PStopMessage | null;
  }>({
    timestamp: 0,
    msg: null,
  });

  const [error, setError] = useState<string | null>(null);

  // TODO(troy): Remove throttled heartbeat?
  const throttledHeartbeat = useThrottle(() => {
    const event = new CustomEvent("pstop-heartbeat");
    window.dispatchEvent(event);
  }, 100);

  const togglePStop = useCallback(() => {
    if (!pStopClient) {
      return;
    }
    pStopClient.togglePStop();
  }, [pStopClient]);

  const attemptConnection = (): PStopClient | undefined => {
    try {
      setError(null);
      const client = new PStopClient({
        receiverId: "1111",
      });
      setClient(client);
      client.addMessageListener((msg) => {
        if (client.isAborted()) {
          return;
        }
        setLastMessage({ timestamp: Date.now(), msg });
        throttledHeartbeat();
      });

      client.addStatusListener((status) => {
        // console.log("status", status);
        // console.trace();
        if (client.isAborted()) {
          return;
        }
        setPStopState(status);
      });

      client
        .activate()
        .then()
        .catch((e) => {
          console.error(e);
          setError(e);
        });

      return client;
    } catch (e) {
      console.error("error in attemptConnection", e);
      setError(e.toString());
    }
  };

  const deactivate = () => {
    if (pStopClient) {
      pStopClient.deactivate();
      setClient(undefined);
    }
  };

  useEffect(() => {
    const client = attemptConnection();
    return () => {
      if (client) {
        try {
          client.abort();
          client
            .deactivate()
            .catch((e) => console.error("Error deactivating client", e));
        } catch (e) {
          console.error(e);
        }
      }
    };
  }, []);

  return (
    <>
      <div className="flex flex-row h-vh w-vw justify-center items-center">
        <div className="flex justify-center items-center w-[500px] h-[500px]">
          <PStopButton pStopState={pStopState} onClick={togglePStop} />
        </div>
        <div className="flex flex-col  w-64 p-5">
          {pStopClient?.isAlive() ? (
            <button onClick={deactivate}>Deactivate</button>
          ) : (
            <button onClick={attemptConnection}>Reconnect</button>
          )}
          <div>Connected: {pStopClient ? "yes" : "no"}</div>
          <div>Status: {getProtectiveStopUI(pStopState).text}</div>
          <div>
            Error:{" "}
            {error ? (
              <span className="text-red-600">{error.toString()}</span>
            ) : (
              "None"
            )}
          </div>
          <div>Last Message</div>
          <div className="border p-2 font-mono text-xs flex flex-col">
            <div>{JSON.stringify(lastMessage, undefined, 2)}</div>
          </div>
        </div>
      </div>
    </>
  );
});

export default PStop;
