defmodule WobblesJr.STM32 do
  @moduledoc """
  Driver for the STM32 power-management coprocessor on the Nerves Starter
  Kit, spoken over the serial link (UART2 / `ttyS2`) at 115200 8N1.

  Commands are serialized: only one is in flight at a time, so the next
  `OK`/`ERR` line the firmware emits is unambiguously the reply to the
  outstanding command. Unsolicited `EVT BTN` events — which the firmware
  may interleave between a command and its reply — are never consumed as a
  reply; they are broadcast to subscribers as

      {:nsk_stm32_event, {:button, pin, high?}}

  Register interest with `subscribe/1`.

  ## Usage

      iex> WobblesJr.STM32.get_power()
      {:ok, %{v_batt_mv: 3812, i_batt_ma: -142, usb_present: true, chrg_en: true}}

      iex> WobblesJr.STM32.set_rtc(System.os_time(:second))
      :ok

  See `WobblesJr.STM32.Protocol` for the wire format, and
  `nsk_stm32/protocol/src/lib.rs` for the firmware-side contract.
  """

  use GenServer
  require Logger

  alias WobblesJr.STM32.Protocol

  @default_port "ttyS2"
  @default_speed 115_200
  @default_timeout 2_000

  # Bounds the firmware enforces, checked client-side so a bad value fails
  # fast without a round trip. See RTC_EPOCH_MIN/MAX and the u32/i32 fields
  # in the protocol crate.
  @rtc_epoch_min 946_684_800
  @rtc_epoch_max 4_102_444_799
  @u32_max 4_294_967_295
  @i32_min -2_147_483_648
  @i32_max 2_147_483_647

  # --- Public API ---

  @doc """
  Start the driver.

  Options:

    * `:port` - serial device name (default `"ttyS2"`)
    * `:speed` - baud rate (default `115200`)
    * `:name` - registered name (default `#{inspect(__MODULE__)}`; pass `nil`
      to leave it unregistered)
    * `:timeout` - per-command reply watchdog in ms (default `2000`)
    * `:subscribers` - initial list of pids to receive events
    * `:uart_module` / `:uart_pid` - transport injection points for tests
  """
  def start_link(opts \\ []) do
    {name, opts} = Keyword.pop(opts, :name, __MODULE__)

    if name do
      GenServer.start_link(__MODULE__, opts, name: name)
    else
      GenServer.start_link(__MODULE__, opts)
    end
  end

  @doc "Subscribe `pid` (default the caller) to `{:nsk_stm32_event, ...}` messages."
  def subscribe(server \\ __MODULE__, pid \\ self()) do
    GenServer.call(server, {:subscribe, pid})
  end

  @doc "Read battery/charger telemetry: `%{v_batt_mv, i_batt_ma, usb_present, chrg_en}`."
  def get_power(server \\ __MODULE__) do
    case command(server, :get_power) do
      {:ok, {:power, metrics}} -> {:ok, metrics}
      other -> other
    end
  end

  @doc "Read the RTC as Unix seconds (UTC)."
  def get_rtc(server \\ __MODULE__) do
    case command(server, :get_rtc) do
      {:ok, {:rtc, epoch}} -> {:ok, epoch}
      other -> other
    end
  end

  @doc "Set the RTC to a Unix timestamp (seconds, UTC), 2000..2099."
  def set_rtc(server \\ __MODULE__, epoch) when is_integer(epoch) do
    if epoch in @rtc_epoch_min..@rtc_epoch_max do
      ack(command(server, {:set_rtc, epoch}))
    else
      {:error, :out_of_range}
    end
  end

  @doc "Ask the coprocessor to cut MPU power and wake again after `seconds`."
  def sleep(server \\ __MODULE__, seconds) when is_integer(seconds) do
    if seconds in 0..@u32_max do
      ack(command(server, {:sleep, seconds}))
    else
      {:error, :out_of_range}
    end
  end

  @doc "Read why the system last woke: `:power_on | :button | :rtc | :unknown`."
  def get_wake_reason(server \\ __MODULE__) do
    case command(server, :get_wake_reason) do
      {:ok, {:wake_reason, reason}} -> {:ok, reason}
      other -> other
    end
  end

  @doc "Set the charge-maintenance voltage dead band, in millivolts (`min <= max`)."
  def set_charge_thresholds(server \\ __MODULE__, min_mv, max_mv)
      when is_integer(min_mv) and is_integer(max_mv) do
    if min_mv in @i32_min..@i32_max and max_mv in @i32_min..@i32_max and min_mv <= max_mv do
      ack(command(server, {:set_charge_thresholds, min_mv, max_mv}))
    else
      {:error, :out_of_range}
    end
  end

  @doc "Enable or disable battery charging."
  def set_charging(server \\ __MODULE__, on?) when is_boolean(on?) do
    ack(command(server, {:set_charging, on?}))
  end

  @doc "Reboot the coprocessor into its DFU bootloader."
  def enter_bootloader(server \\ __MODULE__) do
    ack(command(server, :enter_bootloader))
  end

  # Any `OK` ack collapses to `:ok`; errors and timeouts pass through.
  defp ack({:ok, _}), do: :ok
  defp ack(other), do: other

  defp command(server, cmd) do
    GenServer.call(server, {:command, cmd}, :infinity)
  end

  # --- GenServer ---

  @impl true
  def init(opts) do
    port = Keyword.get(opts, :port, @default_port)
    speed = Keyword.get(opts, :speed, @default_speed)
    timeout = Keyword.get(opts, :timeout, @default_timeout)
    uart_module = Keyword.get(opts, :uart_module, Circuits.UART)
    subscribers = opts |> Keyword.get(:subscribers, []) |> MapSet.new()

    with {:ok, uart} <- open_uart(uart_module, Keyword.get(opts, :uart_pid), port, speed) do
      state = %{
        uart_module: uart_module,
        uart: uart,
        port: port,
        timeout: timeout,
        pending: nil,
        subscribers: subscribers
      }

      {:ok, state}
    end
  end

  defp open_uart(uart_module, nil, port, speed) do
    with {:ok, uart} <- uart_module.start_link() do
      case configure(uart_module, uart, port, speed) do
        :ok -> {:ok, uart}
        error -> {:stop, error}
      end
    end
  end

  defp open_uart(uart_module, uart, port, speed) do
    case configure(uart_module, uart, port, speed) do
      :ok -> {:ok, uart}
      error -> {:stop, error}
    end
  end

  defp configure(uart_module, uart, port, speed) do
    uart_module.open(uart, port,
      speed: speed,
      data_bits: 8,
      stop_bits: 1,
      parity: :none,
      active: true,
      framing: {Circuits.UART.Framing.Line, separator: "\r\n"}
    )
  end

  @impl true
  def handle_call({:subscribe, pid}, _from, state) do
    Process.monitor(pid)
    {:reply, :ok, %{state | subscribers: MapSet.put(state.subscribers, pid)}}
  end

  def handle_call({:command, _cmd}, _from, %{pending: pending} = state)
      when pending != nil do
    # Commands are serialized; refuse to overlap rather than corrupt correlation.
    {:reply, {:error, :busy}, state}
  end

  def handle_call({:command, cmd}, from, state) do
    :ok = state.uart_module.write(state.uart, Protocol.encode(cmd))
    ref = make_ref()
    timer = Process.send_after(self(), {:command_timeout, ref}, state.timeout)
    {:noreply, %{state | pending: %{from: from, ref: ref, timer: timer}}}
  end

  @impl true
  def handle_info({:circuits_uart, port, {:error, reason}}, %{port: port} = state) do
    Logger.warning("STM32 UART error on #{port}: #{inspect(reason)}")
    {:noreply, state}
  end

  def handle_info({:circuits_uart, port, line}, %{port: port} = state)
      when is_binary(line) do
    {:noreply, handle_line(Protocol.decode(line), line, state)}
  end

  def handle_info({:command_timeout, ref}, %{pending: %{ref: ref, from: from}} = state) do
    GenServer.reply(from, {:error, :timeout})
    {:noreply, %{state | pending: nil}}
  end

  def handle_info({:command_timeout, _stale_ref}, state) do
    {:noreply, state}
  end

  def handle_info({:DOWN, _mref, :process, pid, _reason}, state) do
    {:noreply, %{state | subscribers: MapSet.delete(state.subscribers, pid)}}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  # An event is always broadcast and never counts as a command reply, even
  # when a command is outstanding.
  defp handle_line({:event, event}, _line, state) do
    broadcast(state.subscribers, event)
    state
  end

  # An OK/ERR line completes the outstanding command, if there is one.
  defp handle_line({:ok, _} = reply, _line, %{pending: %{} = pending} = state) do
    reply_pending(pending, reply, state)
  end

  defp handle_line({:error, _} = reply, _line, %{pending: %{} = pending} = state) do
    reply_pending(pending, reply, state)
  end

  # A reply with nothing waiting, or an undecodable line: log and move on.
  defp handle_line({:unknown, line}, _line, state) do
    Logger.warning("STM32 unrecognized line: #{inspect(line)}")
    state
  end

  defp handle_line(reply, _line, state) do
    Logger.warning("STM32 unsolicited reply with no pending command: #{inspect(reply)}")
    state
  end

  defp reply_pending(%{from: from, timer: timer}, reply, state) do
    Process.cancel_timer(timer)
    GenServer.reply(from, reply)
    %{state | pending: nil}
  end

  defp broadcast(subscribers, event) do
    Enum.each(subscribers, fn pid -> send(pid, {:nsk_stm32_event, event}) end)
  end
end
