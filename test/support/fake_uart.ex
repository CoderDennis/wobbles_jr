defmodule WobblesJr.STM32.FakeUART do
  @moduledoc """
  In-memory stand-in for `Circuits.UART`, used to drive `WobblesJr.STM32`
  in tests without real hardware.

  It implements only the small subset of the Circuits.UART API that the
  driver calls. Every `write/2` is forwarded to the test process as
  `{:uart_write, binary}`, and incoming lines are simulated by sending
  `{:circuits_uart, port, line}` straight to the driver process, exactly
  as Circuits.UART would in active mode with line framing.
  """

  use Agent

  def start_link(test_pid) do
    Agent.start_link(fn -> %{test_pid: test_pid} end)
  end

  def open(_uart, _name, _opts), do: :ok
  def configure(_uart, _opts), do: :ok
  def close(_uart), do: :ok

  def write(uart, data) do
    test_pid = Agent.get(uart, & &1.test_pid)
    send(test_pid, {:uart_write, IO.iodata_to_binary(data)})
    :ok
  end

  def write(uart, data, _timeout), do: write(uart, data)
end
