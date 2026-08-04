import {
  Channel,
  ChannelId,
  FoxgloveClient,
  Service,
  ServiceCallPayload,
} from "@foxglove/ws-protocol";
import { v4 as uuidv4 } from "uuid";

import { parse as parseMessageDefinition } from "@foxglove/rosmsg";
import { buildPstopMessage } from "./messages";
import {
  ConnectionStatus,
  ProtectiveStopUIState,
  PStopMessage,
  PStopServiceRequest,
  PStopServiceResponse,
} from "./types";
import {
  MessageWriter as Ros2MessageWriter,
  MessageReader as Ros2MessageReader,
} from "@foxglove/rosmsg2-serialization";

export class PStopClient {
  client: FoxgloveClient | undefined;
  receiverId: string;
  senderId: string;
  _pStopPressed: boolean = true;
  _ui_state: ProtectiveStopUIState | undefined;

  _abortController: AbortController = new AbortController();
  _pStopChannel: Channel | undefined;
  _pStopMsgWriter: Ros2MessageWriter | undefined;
  _pStopMsgReader: Ros2MessageReader | undefined;

  _pStopActivationService: Service | undefined;
  _pStopDeactivationService: Service | undefined;
  _pStopServiceRequestWriter: Ros2MessageWriter | undefined;
  _pStopServiceResponseReader: Ros2MessageReader | undefined;

  _pubId: ChannelId | undefined;
  _heartbeatIntervalId: number | undefined;
  _watchdogIntervalId: number | undefined;

  _msgBuffer: PStopMessage[] = [];
  _messageListener: ((msg: PStopMessage) => void) | undefined;
  _statusListener: ((status: ProtectiveStopUIState) => void) | undefined;

  _bufferSize = 10;
  _lastMsgTimestamp: number | undefined;

  constructor({
    receiverId,
    senderId = uuidv4(),
  }: {
    receiverId: string;
    senderId?: string;
  }) {
    this.receiverId = receiverId;
    console.log("SenderId", senderId);
    this.senderId = senderId;

    // console.log("PStopClient constructor");
  }

  // TODO(troy): Add a disconnect timeout param
  async _connectToBridge() {
    return new Promise<void>((resolve, reject) => {
      //  TODO(troy): Make this configurable
      const url = "ws://localhost:8765";

      try {
        this.client = new FoxgloveClient({
          // Check whether env is in nodejs or browser
          ws: new WebSocket(url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL]),
          // ws: !process.env.NODE_ENV ?  new WebSocket(url, [FoxgloveClient.SUPPORTED_SUBPROTOCOL]),
        });
      } catch (e) {
        console.error("Failed to create client: ", e);
        reject(e);
        return;
      }

      const tryResolve = () => {
        if (
          this._pStopChannel &&
          this._pStopActivationService &&
          this._pStopDeactivationService
        ) {
          resolve();
        }
      };

      this.client.on("open", () => {
        // console.log("connected to %s", url);
        // done.resolve();
      });

      this.client.on("advertise", (channels) => {
        const regex = new RegExp(/^protective_stop_msg\/msg\/ProtectiveStop$/);
        const channel = channels.filter((c) => regex.test(c.schemaName));

        if (!channel.length) {
          reject(new Error("ProtectiveStop channel not found"));
          return;
        }

        console.log("Channel", channel);
        // TODO(troy): What if there are multiple channels?
        this._pStopChannel = channel[0];
        const msgDef = parseMessageDefinition(this._pStopChannel.schema, {
          ros2: true,
        });
        this._pStopMsgWriter = new Ros2MessageWriter(msgDef);
        this._pStopMsgReader = new Ros2MessageReader(msgDef);

        tryResolve();
      });

      this.client.on("advertiseServices", (channels) => {
        const regex = new RegExp("protective_stop_msg/srv/");
        const services = channels.filter((s) => regex.test(s.type));
        if (!services.length) {
          reject(new Error("No Protective Stop service not found"));
          return;
        }

        this._pStopActivationService = services.find((s) =>
          /\/activate/.test(s.name)
        );
        this._pStopDeactivationService = services.find((s) =>
          /\/deactivate/.test(s.name)
        );

        if (!this._pStopActivationService || !this._pStopDeactivationService) {
          reject(new Error("ProtectiveStop service not found"));
          return;
        }

        if (
          this._pStopActivationService.requestSchema !==
          this._pStopDeactivationService.requestSchema
        ) {
          reject(new Error("ProtectiveStop service request schema mismatch"));
          return;
        }

        if (
          this._pStopActivationService.responseSchema !==
          this._pStopDeactivationService.responseSchema
        ) {
          reject(new Error("ProtectiveStop service response schema mismatch"));
          return;
        }

        this._pStopServiceRequestWriter = new Ros2MessageWriter(
          parseMessageDefinition(this._pStopActivationService.requestSchema, {
            ros2: true,
          })
        );
        this._pStopServiceResponseReader = new Ros2MessageReader(
          parseMessageDefinition(this._pStopActivationService.responseSchema, {
            ros2: true,
          })
        );

        tryResolve();
      });

      this.client.on("error", (error) => {
        reject(error);
      });
    });
  }

  async call_service(req: PStopServiceRequest, activate: boolean) {
    return new Promise<PStopServiceResponse>((resolve, reject) => {
      if (!this.client) {
        reject(new Error("Client not connected"));
        return;
      }
      if (!this._pStopActivationService || !this._pStopDeactivationService) {
        reject(new Error("ProtectiveStop service definition not found"));
        return;
      }

      if (
        !this._pStopServiceRequestWriter ||
        !this._pStopServiceResponseReader
      ) {
        reject(new Error("ProtectiveStop service writer/reader not found"));
        return;
      }

      this.client.on("serviceCallResponse", (response) => {
        if (
          response.serviceId !== this._pStopActivationService?.id &&
          activate
        ) {
          return;
        }
        if (
          response.serviceId !== this._pStopDeactivationService?.id &&
          !activate
        ) {
          return;
        }

        const decoded =
          this._pStopServiceResponseReader?.readMessage<PStopServiceResponse>(
            response.data
          );
        if (!decoded) {
          reject(new Error("Failed to decode service response"));
          return;
        }
        // console.log("decoded", decoded);
        if (!decoded.success) {
          reject(decoded.message);
        }
        resolve(decoded);
      });

      const requestData = this._pStopServiceRequestWriter.writeMessage(req);

      const request: ServiceCallPayload = {
        serviceId: activate
          ? this._pStopActivationService?.id
          : this._pStopDeactivationService?.id,
        callId: 1,
        encoding: "cdr",
        data: new DataView(requestData.buffer),
      };
      this.client.sendServiceCallRequest(request);
    });
  }

  _startHeartbeat(timeout_ms: number) {
    if (this._heartbeatIntervalId) {
      return;
    }

    this._pubId = this.client.advertise({
      topic: "/protective_stop",
      encoding: "cdr",
      schemaName: "protective_stop_msg/msg/ProtectiveStop",
    });

    const id = setInterval(() => {
      if (!this.client || !this._pStopChannel) {
        return;
      }
      const msg = buildPstopMessage(
        this.receiverId,
        this.senderId,
        this._pStopPressed
      );

      const encoded = this._pStopMsgWriter?.writeMessage(msg);
      this.client.sendMessage(this._pubId, encoded);
    }, timeout_ms);
    this._heartbeatIntervalId = id;
  }

  _stopHeartbeat() {
    if (this._heartbeatIntervalId) {
      clearInterval(this._heartbeatIntervalId);
      this._heartbeatIntervalId = undefined;
    }
  }

  async _startWatchdog(heartbeat_ms: number) {
    if (this._watchdogIntervalId) {
      clearInterval(this._watchdogIntervalId);
    }

    this._lastMsgTimestamp = Date.now();
    this._watchdogIntervalId = setInterval(() => {
      if (this._lastMsgTimestamp) {
        const now = Date.now();
        const delta = now - this._lastMsgTimestamp;
        if (delta > heartbeat_ms) {
          // console.error(
          // `Missed heartbeat. Last message received ${delta} ms ago.`
          // );
          this._setPStopUIState(
            ProtectiveStopUIState.UNSTABLE_ROBOT_CONNECTION
          );
        } else {
          this._setPStopPressed(this._pStopPressed);
        }
      }
    }, heartbeat_ms);
  }

  async _stopWatchdog() {
    if (this._watchdogIntervalId) {
      clearInterval(this._watchdogIntervalId);
      this._watchdogIntervalId = undefined;
    }
  }

  async activate(): Promise<PStopServiceResponse> {
    this._setPStopUIState(ProtectiveStopUIState.CONNECTING);
    try {
      await this._connectToBridge();
    } catch (e) {
      this._setPStopUIState(ProtectiveStopUIState.FAILED_CONNECTION);
      throw new Error(`Failed to connect to bridge: ${e}`);
    }
    const req: PStopServiceRequest = {
      version: 0,
      recv_uuid: this.receiverId,
      sender_uuid: this.senderId,
    };

    try {
      const resp = await this.call_service(req, true);
      if (!resp.success) {
        this._setPStopUIState(ProtectiveStopUIState.FAILED_CONNECTION);
        throw new Error(resp.message);
      }

      if (!this._pStopChannel) {
        throw new Error("ProtectiveStop channel not found");
      }

      const subId = this.client?.subscribe(this._pStopChannel?.id);
      this.client?.on("message", (event) => {
        if (event.subscriptionId === subId && event.data) {
          const decoded = this._pStopMsgReader.readMessage<PStopMessage>(
            event.data
          );
          // Filter out own messages
          if (decoded.receiver_uuid !== this.senderId) {
            return;
          }

          this._lastMsgTimestamp = Date.now();
          if (decoded.connection_status.status === ConnectionStatus.ACTIVE) {
            this._setPStopPressed(decoded.pstop_pressed);
          } else if (
            decoded.connection_status.status === ConnectionStatus.UNSTABLE
          ) {
            this._setPStopUIState(
              ProtectiveStopUIState.UNSTABLE_CLIENT_CONNECTION
            );
          }

          if (this._messageListener) {
            this._messageListener(decoded);
          }
          this._msgBuffer.push(decoded);
          if (this._msgBuffer.length > this._bufferSize) {
            this._msgBuffer.shift();
          }
        }
      });

      this._startHeartbeat(resp.params.heartbeat_timeout * 1000);
      this._startWatchdog(resp.params.heartbeat_timeout * 1000);
      if (this._abortController.signal.aborted) {
        console.error(
          `Aborted during activation. Heartbeat interval id: ${this._heartbeatIntervalId}`
        );
        await this.deactivate();
      }
      this._setPStopPressed(false);
      return resp;
    } catch (e) {
      this._setPStopUIState(ProtectiveStopUIState.FAILED_CONNECTION);
      throw new Error(`Failed to activate: ${e}`);
    }
  }

  addMessageListener(callback: (msg: PStopMessage) => void) {
    this._messageListener = callback;
  }

  togglePStop() {
    if (this._ui_state === ProtectiveStopUIState.ACTIVE_UNPRESSED) {
      this._setPStopPressed(true);
    } else if (this._ui_state === ProtectiveStopUIState.ACTIVE_PRESSED) {
      this._setPStopPressed(false);
    }
  }

  _setPStopPressed(pressed: boolean) {
    this._pStopPressed = pressed;
    this._setPStopUIState(
      pressed
        ? ProtectiveStopUIState.ACTIVE_PRESSED
        : ProtectiveStopUIState.ACTIVE_UNPRESSED
    );
  }

  _setPStopUIState(ui_state: ProtectiveStopUIState) {
    const prevUiState = this._ui_state;
    this._ui_state = ui_state;
    if (this._statusListener && prevUiState !== ui_state) {
      this._statusListener(ui_state);
    }
  }

  addStatusListener(callback: (status: ProtectiveStopUIState) => void) {
    this._statusListener = callback;
  }

  // TODO(troy): Test me!
  isAlive() {
    return this._heartbeatIntervalId !== undefined;
  }

  isAborted() {
    return this._abortController.signal.aborted;
  }

  abort = () => {
    this._abortController.abort();
  };

  async deactivate() {
    // TODO(troy): Add "dicsonnecting..." state?
    this._setPStopUIState(ProtectiveStopUIState.DISCONNECTED);
    // console.log("Deactivating");
    this._stopHeartbeat();
    this._stopWatchdog();

    const req: PStopServiceRequest = {
      version: 0,
      recv_uuid: this.receiverId,
      sender_uuid: this.senderId,
    };
    const resp = await this.call_service(req, false);
    return resp;
  }

  // disconnect() {
  // this.client?.close();
  // }
}
