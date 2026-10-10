defmodule WobblesJr.Leds do
  @moduledoc """
  The four NeoPixels: the robot's safety state, or the battery while it charges.

  Stands in for `BB.NSK.Leds.Controller` and shows exactly what it does —
  every pixel the colour of the operational state, breathing or flashing —
  except while the robot is disarmed on USB power. Then the pixels are a
  charge bar: one per quarter of the way from `empty` to `full` volts, solid
  green for each quarter done, the one being filled breathing, and all four
  solid once the charger has finished. Amber instead of green when USB is
  present but charging is switched off.

  The bar only ever replaces the disarmed colour, so anything that could move
  still shows as it always did.

  The battery comes from `WobblesJr.Sensor.Power` on `[:sensor, :power]`.
  """

  use BB.Controller,
    options_schema: [
      brightness: [
        type: :non_neg_integer,
        default: 40,
        doc: "Brightness at the top of a pulse, and of a lit bar pixel"
      ],
      pulse_period: [
        type: :pos_integer,
        default: 2_000,
        doc: "Milliseconds for a full breath, in and out"
      ],
      flash_period: [
        type: :pos_integer,
        default: 250,
        doc: "Milliseconds for a full flash while fallen"
      ],
      pulse_interval: [
        type: :pos_integer,
        default: 50,
        doc: "Milliseconds between brightness updates"
      ],
      empty: [
        type: :float,
        default: 3.3,
        doc: "Battery volts shown as an empty bar"
      ],
      full: [
        type: :float,
        default: 4.2,
        doc: "Battery volts shown as a full bar"
      ],
      done_current: [
        type: :float,
        default: 0.05,
        doc: "Charge current, in amps, below which a full battery counts as finished"
      ]
    ]

  alias BB.{Message, PubSub}
  alias BB.Message.Sensor.BatteryState
  alias BB.NSK.Leds
  alias BB.NSK.Leds.Controller, as: Stock
  alias BB.Robot.Runtime
  alias BB.StateMachine.Transition

  require Logger

  @green {0, 255, 0}
  @amber {255, 120, 0}
  @off {0, 0, 0}

  @doc """
  What each pixel should show for a battery, from `indicator-0` up: a list of
  `{colour, :solid | :breathe | :off}`, or `nil` when the battery isn't on USB
  and the state colours should show instead.

  The bar fills from `indicator-3`, which is where it reads as filling from on
  this robot.
  """
  @spec bar(BatteryState.t() | nil, keyword) :: [{Leds.colour(), :solid | :breathe | :off}] | nil
  def bar(%BatteryState{power_supply_status: status} = battery, opts)
      when status in [:charging, :not_charging, :full] do
    colour = if status == :not_charging, do: @amber, else: @green
    empty = Keyword.fetch!(opts, :empty)
    full = Keyword.fetch!(opts, :full)
    fraction = ((battery.voltage - empty) / (full - empty)) |> max(0.0) |> min(1.0)

    done? =
      status == :full or
        (fraction >= 1.0 and battery.current < Keyword.fetch!(opts, :done_current))

    # Filling up: whole quarters are solid, and the one in progress breathes.
    lit = if done?, do: 4, else: trunc(fraction * 4)

    for index <- 0..3, place = 3 - index do
      cond do
        place < lit -> {colour, :solid}
        place == lit and status != :not_charging -> {colour, :breathe}
        true -> {@off, :off}
      end
    end
  end

  def bar(_battery, _opts), do: nil

  @impl BB.Controller
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)

    case Leds.open_all() do
      [] ->
        Logger.info("No NeoPixels found, so #{inspect(__MODULE__)} is not starting.")
        :ignore

      [pixel | _rest] = pixels ->
        Process.flag(:trap_exit, true)
        {:ok, _} = PubSub.subscribe(robot, [:state_machine])
        {:ok, _} = PubSub.subscribe(robot, [:sensor, :power])
        Enum.each(pixels, &Leds.release(&1.path))
        :timer.send_interval(Keyword.fetch!(opts, :pulse_interval), :tick)

        max = Leds.max_brightness(pixel.path) || 255

        {:ok,
         %{
           pixels: pixels,
           colours: List.duplicate(nil, length(pixels)),
           operational: Runtime.state(robot),
           battery: nil,
           opts: opts,
           max_brightness: max,
           brightness: brightness(opts, max),
           started_at: System.monotonic_time(:millisecond)
         }}
    end
  end

  @impl BB.Controller
  def handle_info({:bb, _path, %Message{payload: %Transition{to: to}}}, state) do
    {:noreply, %{state | operational: to}}
  end

  def handle_info({:bb, _path, %Message{payload: %BatteryState{} = battery}}, state) do
    {:noreply, %{state | battery: battery}}
  end

  def handle_info(:tick, state), do: {:noreply, render(state)}

  def handle_info(_message, state), do: {:noreply, state}

  @impl BB.Controller
  def handle_options(opts, state) do
    {:ok, %{state | opts: opts, brightness: brightness(opts, state.max_brightness)}}
  end

  defp brightness(opts, max), do: opts |> Keyword.fetch!(:brightness) |> min(max) |> max(0)

  @impl BB.Controller
  def terminate(_reason, _state) do
    Leds.off()
    :ok
  end

  defp render(state) do
    elapsed = System.monotonic_time(:millisecond) - state.started_at
    looks = looks(state, elapsed)

    # Colour costs a sysfs write, so only when it changes; brightness is cheap
    # and skipped by `Leds` when unchanged.
    colours =
      Enum.zip_with([state.pixels, state.colours, looks], fn [pixel, was, {colour, _level}] ->
        if colour != was, do: Leds.set_colour(pixel, colour)
        colour
      end)

    pixels =
      Enum.zip_with(state.pixels, looks, fn pixel, {_colour, level} ->
        Leds.set_brightness(pixel, level)
      end)

    %{state | pixels: pixels, colours: colours}
  end

  # `{colour, brightness}` per pixel.
  defp looks(%{operational: :disarmed} = state, elapsed) do
    case bar(state.battery, state.opts) do
      nil -> state_looks(state, elapsed)
      bar -> Enum.map(bar, &bar_look(&1, state, elapsed))
    end
  end

  defp looks(state, elapsed), do: state_looks(state, elapsed)

  defp bar_look({colour, :solid}, state, _elapsed), do: {colour, state.brightness}
  defp bar_look({colour, :breathe}, state, elapsed), do: {colour, breathe(state, elapsed)}
  defp bar_look({colour, :off}, _state, _elapsed), do: {colour, 0}

  defp state_looks(state, elapsed) do
    {colour, profile} = Stock.pattern(state.operational)

    level =
      case profile do
        :flash -> Stock.flash(elapsed, state.brightness, dimmest(state), flash_period(state))
        :breathe -> breathe(state, elapsed)
      end

    List.duplicate({colour, level}, length(state.pixels))
  end

  defp breathe(state, elapsed) do
    Stock.breath(
      elapsed,
      state.brightness,
      dimmest(state),
      Keyword.fetch!(state.opts, :pulse_period)
    )
  end

  defp flash_period(state), do: Keyword.fetch!(state.opts, :flash_period)

  # A breath bottoms out at a glow, so it never reads as a pixel gone dark.
  defp dimmest(state), do: div(state.brightness, 8)
end
