import React, { use, useEffect } from "react";
import PStopButton from "./PStopButton";
import { ProtectiveStopUIState } from "./types";

export default {
  title: "Components/PStopButton",
  component: PStopButton,
};

const Template = (args) => <PStopButton {...args} />;

export const Default = Template.bind({});
Default.args = {};

export const PressHandlerHeartbeat = () => {
  const [pressed, setPressed] = React.useState(false);
  const triggerHeartbeat = () => {
    const event = new CustomEvent("pstop-heartbeat");
    window.dispatchEvent(event);
  };

  useEffect(() => {
    const interval = setInterval(async () => {
      triggerHeartbeat();
    }, 100);
    return () => {
      clearInterval(interval);
    };
  }, []);

  return (
    <div className="flex justify-center items-center w-[500px] h-[500px]">
      <PStopButton
        pStopState={
          pressed
            ? ProtectiveStopUIState.ACTIVE_PRESSED
            : ProtectiveStopUIState.ACTIVE_UNPRESSED
        }
        onClick={() => setPressed((prev) => !prev)}
      />
    </div>
  );
};

export const WithSlowHeartbeat = () => {
  const triggerHeartbeat = () => {
    const event = new CustomEvent("pstop-heartbeat");
    window.dispatchEvent(event);
  };

  useEffect(() => {
    const interval = setInterval(() => {
      triggerHeartbeat();
    }, 1000);
    return () => {
      clearInterval(interval);
    };
  }, []);

  return (
    <div className="flex justify-center items-center w-[500px] h-[500px]">
      <PStopButton pStopState={ProtectiveStopUIState.ACTIVE_UNPRESSED} />
    </div>
  );
};
export const ResizedContainerLarge = () => {
  return (
    <div className="flex justify-center items-center w-[1000px] h-[1000px]">
      <PStopButton pStopState={ProtectiveStopUIState.ACTIVE_UNPRESSED} />
    </div>
  );
};

export const ResizedContainerSmall = () => {
  return (
    <div className="flex justify-center items-center w-[200px] h-[200px]">
      <PStopButton pStopState={ProtectiveStopUIState.ACTIVE_UNPRESSED} />
    </div>
  );
};

export const Disconnected = () => {
  return (
    <div className="flex justify-center items-center w-[200px] h-[200px]">
      <PStopButton pStopState={ProtectiveStopUIState.DISCONNECTED} />
    </div>
  );
};

export const Connecting = () => {
  const triggerHeartbeat = () => {
    const event = new CustomEvent("pstop-heartbeat");
    window.dispatchEvent(event);
  };

  useEffect(() => {
    const interval = setInterval(() => {
      triggerHeartbeat();
    }, 100);
    return () => {
      clearInterval(interval);
    };
  }, []);

  return (
    <div className="flex justify-center items-center w-[200px] h-[200px]">
      <PStopButton pStopState={ProtectiveStopUIState.CONNECTING} />
    </div>
  );
};

export const UnstableConnection = () => {
  const triggerHeartbeat = () => {
    const event = new CustomEvent("pstop-heartbeat");
    window.dispatchEvent(event);
  };

  useEffect(() => {
    const interval = setInterval(() => {
      triggerHeartbeat();
    }, 100);
    return () => {
      clearInterval(interval);
    };
  }, []);

  return (
    <div className="flex justify-center items-center w-[200px] h-[200px]">
      <PStopButton
        pStopState={ProtectiveStopUIState.UNSTABLE_ROBOT_CONNECTION}
      />
    </div>
  );
};

export const FailedConnection = () => {
  return (
    <div className="flex justify-center items-center w-[200px] h-[200px]">
      <PStopButton pStopState={ProtectiveStopUIState.FAILED_CONNECTION} />
    </div>
  );
};

export const PressHandlerWithVariableHeartbeat = () => {
  const [pressed, setPressed] = React.useState(false);
  const triggerHeartbeat = () => {
    const event = new CustomEvent("pstop-heartbeat");
    window.dispatchEvent(event);
  };

  useEffect(() => {
    const interval = setInterval(async () => {
      await new Promise((resolve) => setTimeout(resolve, Math.random() * 500));
      triggerHeartbeat();
    }, 100);
    return () => {
      clearInterval(interval);
    };
  }, []);

  return (
    <div className="flex justify-center items-center w-[500px] h-[500px]">
      <PStopButton
        pStopState={
          pressed
            ? ProtectiveStopUIState.ACTIVE_PRESSED
            : ProtectiveStopUIState.ACTIVE_UNPRESSED
        }
        onClick={() => setPressed((prev) => !prev)}
      />
    </div>
  );
};
