import React, { useState, useEffect } from "react";
import { ProtectiveStopUIState } from "./types";

export const getProtectiveStopUI = (
  state: ProtectiveStopUIState
): {
  text: string;
  primaryColor: string;
  secondaryColor: string;
} => {
  let text: string, primaryColor: string, secondaryColor: string;

  switch (state) {
    case ProtectiveStopUIState.CONNECTING: {
      text = "CONNECTING";
      primaryColor = "#3B82F6"; // blue-500
      secondaryColor = "#93C5FD"; // blue-300
      break;
    }
    case ProtectiveStopUIState.ACTIVE_UNPRESSED: {
      text = "STOP";
      primaryColor = "#EF4444"; // red-500
      secondaryColor = "#FCA5A5"; // red-300
      break;
    }
    case ProtectiveStopUIState.ACTIVE_PRESSED: {
      text = "RELEASE";
      primaryColor = "#10B981"; // green-500
      secondaryColor = "#6EE7B7"; // green-300
      break;
    }
    case ProtectiveStopUIState.UNSTABLE_ROBOT_CONNECTION: {
      text = "UNSTABLE";
      primaryColor = "#F59E0B"; // yellow-500
      secondaryColor = "#FCD34D"; // yellow-300
      break;
    }
    case ProtectiveStopUIState.FAILED_CONNECTION: {
      text = "ERROR";
      primaryColor = "#6B7280"; // gray-500
      secondaryColor = "#D1D5DB"; // gray-300
      break;
    }
    case ProtectiveStopUIState.DISCONNECTED: {
      text = "DISCONNECTED";
      primaryColor = "#6B7280"; // gray-500
      secondaryColor = "#D1D5DB"; // gray-300
      break;
    }
    default: {
      text = "UNKNOWN";
      primaryColor = "#6B7280"; // gray-500
      secondaryColor = "#D1D5DB"; // gray-300
      break;
    }
  }

  return { text, primaryColor, secondaryColor };
};

const noop = () => {};

const Ring = ({
  pStopState,
  onClick = noop,
}: {
  pStopState: ProtectiveStopUIState;
  onClick?: () => void;
}) => {
  const totalDots = 30;
  const [activeCount, setActiveCount] = useState(0);
  const containerRef = React.useRef<HTMLDivElement>(null);
  const [radius, setRadius] = useState(0);
  const { text, primaryColor, secondaryColor } =
    getProtectiveStopUI(pStopState);

  // console.log("pStopState", pStopState);
  const clickable =
    pStopState === ProtectiveStopUIState.ACTIVE_PRESSED ||
    pStopState === ProtectiveStopUIState.ACTIVE_UNPRESSED;
  // Update the radius based on the container's size.
  useEffect(() => {
    const updateRadius = () => {
      if (containerRef.current) {
        const { width, height } = containerRef.current.getBoundingClientRect();
        // Calculate a radius that fits within the container (80% of half the minimum dimension)
        setRadius((Math.min(width, height) / 2) * 0.8);
      }
    };

    // Call on mount and add a resize listener for responsiveness.
    updateRadius();
    window.addEventListener("resize", updateRadius);

    return () => window.removeEventListener("resize", updateRadius);
  }, []);
  useEffect(() => {
    const handleHeartbeat = () => {
      setActiveCount((prevCount) => (prevCount + 1) % (totalDots + 1));
    };

    window.addEventListener("pstop-heartbeat", handleHeartbeat);
    return () => {
      window.removeEventListener("pstop-heartbeat", handleHeartbeat);
    };
  }, []);
  const dotSize = radius * 0.08; // adjust the factor as needed

  const dots = Array.from({ length: totalDots }, (_, index) => {
    const angle = (index / totalDots) * 360;
    const transform = `rotate(${angle}deg) translate(${radius}px) rotate(-${angle}deg)`;
    const style = {
      transform,
      width: `${dotSize}px`,
      height: `${dotSize}px`,
      // Center the dot by offsetting half its size.
      marginLeft: `-${dotSize / 2}px`,
      marginTop: `-${dotSize / 2}px`,
    };

    const numDotsShown = 20;
    const upperBound = activeCount;
    const lowerBound = activeCount - numDotsShown;

    const bgColor = (
      lowerBound < 0
        ? index <= upperBound || index >= lowerBound + totalDots
        : index >= lowerBound && index <= upperBound
    )
      ? primaryColor
      : secondaryColor;

    const dotStyle = {
      ...style,
      backgroundColor:
        pStopState !== ProtectiveStopUIState.ACTIVE_PRESSED &&
        pStopState !== ProtectiveStopUIState.ACTIVE_UNPRESSED
          ? primaryColor
          : bgColor,
      transitionProperty: "background-color",
      transitionDuration: `200ms`,
    };

    return (
      <div
        key={index}
        style={dotStyle}
        className="absolute top-1/2 left-1/2 rounded-full transition-colors"
      />
    );
  });

  /*
fill: #E73D31;
filter: drop-shadow(0px 4px 6.8px rgba(0, 0, 0, 0.25));

"drop-shadow-[0_35px_35px_rgba(0,0,0,0.25)]
  */

  const animation = `transition-colors duration-200`;
  const btn = React.useMemo(() => {
    const Wrapper = ({ children}) =>
      clickable ? (
        <div
          style={{ backgroundColor: primaryColor }}
          className={`flex justify-center items-center w-7/8 h-7/8 hover:scale-102 transition-transform duration-200 rounded-full ${
            pStopState === ProtectiveStopUIState.ACTIVE_UNPRESSED
              ? "filter drop-shadow-[0px_0px_6.8px_rgba(0,0,0,0.25)]"
              : "filter inset-shadow-[0px_0px_6.8px_rgba(0,0,0,0.25)]"
          }`}
        >
          <div className="flex justify-center items-center w-19/20 h-19/20 rounded-full filter drop-shadow-[0px_0px_4px_rgba(255,255,255,0.25)]">
            {children}
          </div>
        </div>
      ) : (
        <>{children}</>
      );
    // console.log("clickable", clickable);

    const textSize = radius * 0.25; // Adjust this factor to scale the text.
    return (
      <div
        onClick={clickable ? onClick : () => {}}
        style={{ backgroundColor: primaryColor }}
        className={`flex justify-center items-center w-5/8 h-5/8 rounded-full ${
          clickable ? "cursor-pointer" : ""
        }`}
      >
        <Wrapper>
          <span
            style={{
              fontSize: `${textSize}px`,
              lineHeight: `${textSize}px`,
            }}
            className="text-white font-mono select-none"
          >
            {text}
          </span>
        </Wrapper>
      </div>
    );
  }, [clickable, onClick, primaryColor, radius, text]);

  return (
    <div
      ref={containerRef}
      className="relative w-full h-full flex justify-center items-center"
    >
      {btn}
      {dots}
    </div>
  );
};

export default Ring;
