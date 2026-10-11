defmodule WobblesJr.Sensor.Power do
  @moduledoc """
  The board's power and buttons, from the STM32 coprocessor, on the robot's
  PubSub.

  Polls `WobblesJr.STM32.get_power/0` every `interval` ms and publishes a
  `BB.Message.Sensor.BatteryState` on `[:sensor, :power]`: voltage and current
  in volts and amps (the current as the STM32 reports it, which stays negative
  whether or not it's charging), whether USB power is present, and a charging
  status from the STM32's USB and charge-enable flags. It also republishes the STM32's button events
  as `WobblesJr.Message.Button` on `[:buttons]`, for whatever wants to act on them.

  `WobblesJr.STM32` owns the serial link and is started by the application, on the
  board only. This looks it up on every poll rather than requiring it at start,
  so start order doesn't matter, a restarted driver is picked up again, and on
  the host (no `ttyS2`) this simply publishes nothing.
  """

  use BB.Sensor,
    options_schema: [
      interval: [
        type: :pos_integer,
        default: 1_000,
        doc: "How often to read the battery, in milliseconds"
      ],
      stm32: [
        type: :atom,
        default: WobblesJr.STM32,
        doc: "The registered name of the STM32 driver"
      ]
    ]

  alias BB.Message.Sensor.BatteryState
  alias WobblesJr.Message.Button

  require Logger

  @buttons_path [:buttons]

  @doc "Where button events are published."
  def buttons_path, do: @buttons_path

  @impl BB.Sensor
  def init(opts) do
    %{robot: robot, path: path} = Keyword.fetch!(opts, :bb)
    :timer.send_interval(Keyword.fetch!(opts, :interval), :read)

    {:ok,
     %{
       robot: robot,
       path: path,
       stm32: Keyword.fetch!(opts, :stm32),
       subscribed_to: nil,
       warned?: false
     }}
  end

  @impl BB.Sensor
  def handle_info(:read, state), do: {:noreply, state |> subscribe() |> read()}

  def handle_info({:nsk_stm32_event, {:button, pin, pressed?}}, state) when pin in 0..2 do
    {:ok, message} = Button.new(:base_link, button: pin + 1, pressed: pressed?)
    BB.PubSub.publish(state.robot, @buttons_path, message)
    {:noreply, state}
  end

  def handle_info(_message, state), do: {:noreply, state}

  # Subscribe to button events once per driver process, so a restarted driver
  # is subscribed to again.
  defp subscribe(state) do
    case Process.whereis(state.stm32) do
      nil ->
        state

      pid when pid == state.subscribed_to ->
        state

      pid ->
        :ok = WobblesJr.STM32.subscribe(state.stm32)
        %{state | subscribed_to: pid}
    end
  catch
    :exit, _ -> state
  end

  defp read(%{subscribed_to: nil} = state), do: state

  defp read(state) do
    case WobblesJr.STM32.get_power(state.stm32) do
      {:ok, power} ->
        {:ok, message} = BatteryState.new(:base_link, battery_fields(power))
        BB.PubSub.publish(state.robot, [:sensor | state.path], message)
        %{state | warned?: false}

      # The driver serializes commands; skip a beat rather than queue up.
      {:error, :busy} ->
        state

      {:error, reason} ->
        unless state.warned?, do: Logger.warning("Could not read STM32 power: #{inspect(reason)}")
        %{state | warned?: true}
    end
  catch
    :exit, reason ->
      unless state.warned?, do: Logger.warning("STM32 driver unavailable: #{inspect(reason)}")
      %{state | warned?: true}
  end

  @doc false
  def battery_fields(%{v_batt_mv: mv, i_batt_ma: ma, usb_present: usb?, chrg_en: chrg?}) do
    [
      voltage: mv / 1000,
      current: ma / 1000,
      present: true,
      power_supply_status: status(usb?, chrg?)
    ]
  end

  # From the flags, not the current: on Wobbles Jr the STM32 read about -550 mA
  # on battery and the same on USB while the voltage climbed, so its sign doesn't
  # say which way the battery is going.
  defp status(true, true), do: :charging
  defp status(true, false), do: :not_charging
  defp status(false, _chrg?), do: :discharging
end
