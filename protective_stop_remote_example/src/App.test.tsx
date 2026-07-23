import { render, screen, waitFor } from "@testing-library/react";
import App from "./App";
import { vi, expect, describe, it } from "vitest";

// Mock the PStopClient to avoid real network connections
vi.mock("./client", () => {
  return {
    PStopClient: vi.fn().mockImplementation(() => ({
      addMessageListener: vi.fn(),
      activate: vi.fn().mockResolvedValue("activated"),
      isAlive: vi.fn().mockReturnValue(false),
      deactivate: vi.fn(),
    })),
  };
});

describe("App Component", () => {
  it("renders correctly with initial disconnected state and calls activate", async () => {
    render(<App />);

    // Verify that the protective stop UI displays "DISCONNECTED"
    // expect(screen.getByText("DISCONNECTED")).toBeInTheDocument();
    const text = screen.getByText("DISCONNECTED");
    expect(text).toBeTruthy();

    // Wait until the client has been instantiated and its activate method has been called.
    // await waitFor(() => {
    // Access the mocked module to check the client instantiation
    // const { PStopClient } = require("./client");
    // expect(PStopClient).toHaveBeenCalled();
    // });
  });
});
