defmodule WobblesJr.Face do
  @moduledoc """
  Puts a face on the e-paper panel while the robot is up, reacting to how it's
  moving, and bb_nsk's status screen the rest of the time.

  | doing                  | panel         |
  | ---------------------- | ------------- |
  | balancing, still       | `-_-`         |
  | moving forward         | `>.<`         |
  | moving backward        | `O_O`         |
  | turning right          | `O.o`         |
  | turning left           | `o.O`         |
  | fallen                 | `x_x`         |
  | disarmed or idle       | status screen |

  Ported from lawik/goatbot's `Goatbot.Face`, with the status screen in place
  of the idle face.

  Motion is measured, not commanded, so a robot drifting on its own pulls a
  face as much as one being driven:

  - **speed** is the wheel command averaged over both wheels, times the wheel
    radius. There are no encoders, but with the balance loop's integral the
    command tracks the wheel speed — the same estimate
    `WobblesJr.VelocityController` leans against.
  - **turn rate** is the gyro's yaw rate off the `:heading` sensor, positive
    to the left.

  Both are low-passed over `filter` seconds, so the balance loop's wobble
  doesn't flicker the face. Each is scaled by its threshold and whichever is
  further past it wins.

  Only a change of face reaches the panel, and `BB.NSK.Display.FrameSink` only
  ever queues the newest frame, so motion changing faster than the panel can
  refresh costs one refresh of where it ended up. The status screen is redrawn
  as `BB.NSK.Display.Controller` would: once changes to the state or the WiFi
  settle, and on every `refresh_interval`. Partial refreshes leave ghosting, so
  every `refresh_interval` the panel is redrawn with a full one.

  Without a viewport (on the host, where there is no emerge) this runs anyway
  and its screens go nowhere, so the robot's topology is the same everywhere.
  """

  import BB.Unit

  use BB.Controller,
    options_schema: [
      viewport: [
        type: :atom,
        default: BB.NSK.Display.Viewport,
        doc: "The Emerge viewport that renders screens"
      ],
      frame_sink: [
        type: :atom,
        default: BB.NSK.Display.FrameSink,
        doc: "The process that puts rendered frames on the panel"
      ],
      speed_threshold: [
        type: :float,
        default: 0.02,
        doc: "Speed, in m/s, past which it counts as moving"
      ],
      turn_threshold: [
        type: :float,
        default: 0.35,
        doc: "Yaw rate, in rad/s, past which it counts as turning (about 20°/s)"
      ],
      filter: [
        type: :float,
        default: 0.5,
        doc: "Seconds speed and yaw rate are averaged over"
      ],
      wheel_radius: [
        type: BB.Unit.Option.unit_type(compatible: :meter),
        default: ~u(21.5 millimeter),
        doc: "Turns wheel rad/s into m/s"
      ],
      wheels: [
        type: {:list, :atom},
        default: [:left_wheel, :right_wheel],
        doc: "The actuators whose commands it averages. Both, so a turn cancels out"
      ],
      rate: [type: :pos_integer, default: 20, doc: "How often, in Hz, the face is reconsidered"],
      settle: [
        type: :timeout,
        default: :timer.seconds(2),
        doc: "How long to wait for changes to stop arriving before redrawing the status screen"
      ],
      refresh_interval: [
        type: :timeout,
        default: :timer.minutes(5),
        doc: "How often to clear ghosting with a full refresh"
      ]
    ]

  alias BB.Message
  alias BB.Message.Actuator.Command.Velocity
  alias BB.Message.Geometry.Twist2D
  alias BB.Message.Sensor.JointState
  alias BB.NSK.Display.{FrameSink, Viewport}
  alias BB.NSK.Message.EnvironmentState
  alias BB.NSK.Status
  alias BB.Robot.Units
  alias BB.StateMachine.Transition
  alias WobblesJr.Face.Screen

  # The renderer, the sink and the screen only exist where emerge does, and
  # VintageNet only on the target.
  @compile {:no_warn_undefined, [FrameSink, Viewport, Screen, VintageNet]}

  @wlan "wlan0"

  @doc """
  What the panel shows for a robot state and its motion: a face, or `:status`
  for bb_nsk's status screen. Pure, so it can be tested without a panel.

  `forward` and `left` are speed and yaw rate as multiples of their thresholds,
  so either counts once it is past 1. Negative is backward and right.
  """
  @spec face(atom, float, float) :: String.t() | :status
  def face(:balancing, forward, left) do
    cond do
      max(abs(forward), abs(left)) < 1.0 -> "-_-"
      abs(forward) >= abs(left) and forward > 0 -> ">.<"
      abs(forward) >= abs(left) -> "O_O"
      left < 0 -> "O.o"
      true -> "o.O"
    end
  end

  def face(:fallen, _forward, _left), do: "x_x"
  def face(_mode, _forward, _left), do: :status

  @impl BB.Controller
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)

    # An e-paper panel keeps its image unpowered, so blank it on the way out
    # rather than leave a face on a robot that's gone. `terminate/2` only runs
    # if exits arrive as messages.
    Process.flag(:trap_exit, true)

    {:ok, _} = BB.PubSub.subscribe(robot, [:state_machine])
    {:ok, _} = BB.PubSub.subscribe(robot, [:sensor, :environment])

    for wheel <- Keyword.fetch!(opts, :wheels) do
      {:ok, wheel_path} = BB.Robot.actuator_path(robot.robot(), wheel)
      {:ok, _} = BB.PubSub.subscribe(robot, [:actuator | wheel_path])
    end

    {:ok, ground} = BB.Robot.path_to(robot.robot(), :ground)

    {:ok, _} =
      BB.PubSub.subscribe(robot, [:sensor | ground] ++ [:heading], message_types: [JointState])

    if Code.ensure_loaded?(VintageNet), do: VintageNet.subscribe(["interface", @wlan])

    :timer.send_interval(div(1000, Keyword.fetch!(opts, :rate)), :tick)
    :timer.send_interval(Keyword.fetch!(opts, :refresh_interval), :full_refresh)

    state =
      %{
        robot: robot,
        mode: BB.Robot.Runtime.state(robot),
        face: nil,
        last: nil,
        environment: nil,
        settle_timer: nil,
        viewport: Keyword.fetch!(opts, :viewport),
        frame_sink: Keyword.fetch!(opts, :frame_sink),
        speed_threshold: Keyword.fetch!(opts, :speed_threshold),
        turn_threshold: Keyword.fetch!(opts, :turn_threshold),
        filter: Keyword.fetch!(opts, :filter),
        settle: Keyword.fetch!(opts, :settle),
        wheel_radius: Units.to_meters(Keyword.fetch!(opts, :wheel_radius))
      }
      |> Map.merge(still())

    {:ok, state, {:continue, :show}}
  end

  defp still, do: %{sum: 0.0, count: 0, yaw_rate: 0.0, speed: 0.0, turn: 0.0}

  @impl BB.Controller
  def handle_continue(:show, state), do: {:noreply, show(state, :full)}

  @impl BB.Controller
  # The status screen shows the state, so a move between two states that both
  # get it, such as disarmed to idle, still redraws it.
  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{to: to}}}, state) do
    state = show(Map.merge(%{state | mode: to}, still()), :partial)
    {:noreply, if(state.face == :status, do: settle(state), else: state)}
  end

  # Room temperature drifting by a tenth of a degree is not worth a refresh, so
  # this is kept for the next status redraw rather than causing one.
  def handle_info({:bb, _path, %Message{payload: %EnvironmentState{} = environment}}, state) do
    {:noreply, %{state | environment: environment}}
  end

  def handle_info({:bb, [:actuator | _], %Message{payload: %Velocity{velocity: velocity}}}, state) do
    {:noreply, %{state | sum: state.sum + velocity, count: state.count + 1}}
  end

  # A hold or stop is the wheel at rest, which counts as a zero command.
  def handle_info({:bb, [:actuator | _], %Message{}}, state) do
    {:noreply, %{state | count: state.count + 1}}
  end

  def handle_info(
        {:bb, _path, %Message{payload: %JointState{velocities: [%Twist2D{omega: omega}]}}},
        state
      ) do
    {:noreply, %{state | yaw_rate: omega}}
  end

  def handle_info({VintageNet, _property, _old, _new, _meta}, %{face: :status} = state) do
    {:noreply, settle(state)}
  end

  def handle_info(:settled, %{face: :status} = state) do
    draw(state, :status, :partial)
    {:noreply, %{state | settle_timer: nil}}
  end

  def handle_info(:settled, state), do: {:noreply, %{state | settle_timer: nil}}

  def handle_info(:tick, state) do
    now = System.monotonic_time(:nanosecond)
    dt = if state.last, do: (now - state.last) / 1_000_000_000, else: 0.0
    {:noreply, state |> sample(dt) |> Map.put(:last, now) |> show(:partial)}
  end

  # The status screen's uptime and load go stale, so it is redrawn afresh; a
  # face is the same frame again.
  def handle_info(:full_refresh, %{face: :status} = state) do
    draw(state, :status, :full)
    {:noreply, state}
  end

  def handle_info(:full_refresh, state) do
    if Process.whereis(state.frame_sink), do: FrameSink.redraw(state.frame_sink, :full)
    {:noreply, state}
  end

  def handle_info(_message, state), do: {:noreply, state}

  @impl BB.Controller
  def terminate(_reason, state) do
    if Process.whereis(state.frame_sink), do: FrameSink.clear(state.frame_sink)
    :ok
  end

  # Fold the wheel commands since the last tick, and the latest yaw rate, into
  # the filtered speed and turn rate.
  defp sample(state, dt) when dt > 0.0 do
    command = if state.count > 0, do: state.sum / state.count, else: 0.0
    k = dt / (state.filter + dt)
    speed = state.speed + (command * state.wheel_radius - state.speed) * k
    turn = state.turn + (state.yaw_rate - state.turn) * k
    %{state | speed: speed, turn: turn, sum: 0.0, count: 0}
  end

  defp sample(state, _dt), do: %{state | sum: 0.0, count: 0}

  # Only a different face is worth a refresh, except the first, which is full.
  # Arriving at the status screen waits for things to settle, so a burst of
  # transitions costs one refresh.
  defp show(state, refresh_type) do
    face =
      face(state.mode, state.speed / state.speed_threshold, state.turn / state.turn_threshold)

    state =
      cond do
        refresh_type == :full -> tap(state, &draw(&1, face, :full))
        face == state.face -> state
        face == :status -> settle(state)
        true -> tap(state, &draw(&1, face, refresh_type))
      end

    %{state | face: face}
  end

  # Restart the timer on every change so that a burst of them, such as an
  # interface coming up, costs one refresh rather than one each.
  defp settle(state) do
    if state.settle_timer, do: Process.cancel_timer(state.settle_timer)
    %{state | settle_timer: Process.send_after(self(), :settled, state.settle)}
  end

  # The sink is told how to refresh before the screen goes to the renderer, so
  # the frame it gets back is already paired with the right refresh.
  defp draw(state, face, refresh_type) do
    if Process.whereis(state.frame_sink) && Process.whereis(state.viewport) do
      :ok = FrameSink.configure(state.frame_sink, :bw1, refresh_type)
      Viewport.show(state.viewport, screen(state, face))
    end

    :ok
  end

  defp screen(state, :status) do
    state.robot
    |> Status.snapshot()
    |> put_environment(state.environment)
  end

  defp screen(_state, face), do: Screen.screen(face)

  defp put_environment(status, nil), do: status

  defp put_environment(status, environment) do
    %{status | temperature_c: environment.temperature, humidity_percent: environment.humidity}
  end
end
