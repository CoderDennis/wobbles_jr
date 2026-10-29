defmodule WobblesJr.Buttons do
  @moduledoc """
  What the board's three buttons do.

  Seen from the front, the STM32 numbers them right to left:

  - **Left (3), press: stand.** Arms first if the robot isn't armed, like
    `WobblesJr.IExHelpers.go/0`, so one press gets it up. Hold it upright first.
  - **Left (3), double press: explore on or off** (`WobblesJr.Explore`). The
    first press still stands the robot at once, so a double press from idle
    stands it up and sets it exploring as soon as it is balancing.
  - **Middle (2), press: disarm.** A stop that needs no phone or laptop.
  - **Right (1), hold: power off.** Only once it has been held for
    `:hold_time`, so a bump can't do it, and only while disarmed, because the
    `poweroff` command refuses to drop a robot that is balancing.

  Presses come from `WobblesJr.Sensor.Power` on `[:buttons]`. Commands run in
  their own process, since `stand` waits for `arm` and neither should hold up
  the next press.
  """

  use BB.Controller,
    options_schema: [
      hold_time: [
        type: :pos_integer,
        default: 2_000,
        doc: "Milliseconds the power button has to be held"
      ],
      double_press: [
        type: :pos_integer,
        default: 400,
        doc: "Milliseconds within which a second left press counts as a double press"
      ]
    ]

  alias BB.{Message, PubSub}
  alias WobblesJr.IExHelpers
  alias WobblesJr.Message.Button
  alias WobblesJr.Sensor.Power

  require Logger

  @stand 3
  @disarm 2
  @power 1

  @doc """
  What pressing `button` starts: a command to run now, a command to run once
  it has been held, or nothing.
  """
  @spec on_press(1..3) :: {:now, :stand | :disarm} | {:hold, :poweroff}
  def on_press(@stand), do: {:now, :stand}
  def on_press(@disarm), do: {:now, :disarm}
  def on_press(@power), do: {:hold, :poweroff}

  @doc """
  Whether a left press at `now` comes soon enough after the `previous` one, both
  in milliseconds, to make a double press.
  """
  @spec double_press?(integer | nil, integer, pos_integer) :: boolean
  def double_press?(nil, _now, _window), do: false
  def double_press?(previous, now, window), do: now - previous <= window

  @impl BB.Controller
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)
    {:ok, _} = PubSub.subscribe(robot, Power.buttons_path())

    {:ok, %{robot: robot, opts: opts, held: %{}, left_at: nil}}
  end

  @impl BB.Controller
  def handle_info({:bb, _path, %Message{payload: %Button{button: button, pressed: true}}}, state) do
    case on_press(button) do
      {:now, :stand} ->
        now = System.monotonic_time(:millisecond)

        # A third press starts over rather than making another double.
        if double_press?(state.left_at, now, Keyword.fetch!(state.opts, :double_press)) do
          run(state.robot, :explore)
          {:noreply, %{state | left_at: nil}}
        else
          run(state.robot, :stand)
          {:noreply, %{state | left_at: now}}
        end

      {:now, name} ->
        run(state.robot, name)
        {:noreply, state}

      {:hold, name} ->
        timer = Process.send_after(self(), {:held, button, name}, hold_time(state))
        {:noreply, put_in(state.held[button], timer)}
    end
  end

  def handle_info({:bb, _path, %Message{payload: %Button{button: button, pressed: false}}}, state) do
    {timer, held} = Map.pop(state.held, button)
    if timer, do: Process.cancel_timer(timer)
    {:noreply, %{state | held: held}}
  end

  def handle_info({:held, button, name}, state) do
    run(state.robot, name)
    {:noreply, %{state | held: Map.delete(state.held, button)}}
  end

  def handle_info(_message, state), do: {:noreply, state}

  @impl BB.Controller
  def handle_options(opts, state), do: {:ok, %{state | opts: opts}}

  defp hold_time(state), do: Keyword.fetch!(state.opts, :hold_time)

  defp run(robot, name) do
    Task.start(fn ->
      Logger.info("Button: #{name}")

      case command(robot, name) do
        {:error, reason} -> Logger.warning("Button #{name} failed: #{inspect(reason)}")
        _ok -> :ok
      end
    end)
  end

  defp command(_robot, :stand), do: IExHelpers.go()
  defp command(robot, :explore), do: WobblesJr.Explore.toggle(robot)

  defp command(robot, name) do
    with {:ok, pid} <- apply(robot, name, []), do: BB.Command.await(pid, 60_000)
  end
end
