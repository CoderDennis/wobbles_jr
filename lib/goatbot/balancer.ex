defmodule Goatbot.Balancer do
  @moduledoc """
  A minimal PID balance loop.

  Ticks on every sample the `:lean_angle` sensor publishes (a `JointState` with
  the lean in radians and the gyro's lean rate in rad/s), and drives both wheels
  at the same velocity:

      command = kp * error + ki * ∫error dt + kd * rate,  error = lean - setpoint

  It only drives while the robot is `:balancing`. Arm the robot, hold it
  upright, then run `Goatbot.Robot.stand/0` to hand it over. If the lean error
  passes `fall_angle`, it brakes the wheels and runs `Goatbot.Robot.fall/0`.
  """

  use BB.Controller,
    options_schema: [
      setpoint: [
        type: BB.Unit.Option.unit_type(compatible: :degree),
        required: true,
        doc: "The lean to hold. Negative leans back"
      ],
      kp: [type: :float, required: true, doc: "Wheel rad/s per radian of lean error"],
      ki: [type: :float, default: 0.0, doc: "Wheel rad/s per radian-second of lean error"],
      kd: [type: :float, required: true, doc: "Wheel rad/s per rad/s of lean rate"],
      integral_limit: [
        type: :float,
        default: 0.1,
        doc: "Clamp on the integral, in radian-seconds, so it can't wind up"
      ],
      fall_angle: [
        type: BB.Unit.Option.unit_type(compatible: :degree),
        required: true,
        doc: "Lean error past which it gives up"
      ],
      left_wheel: [type: :atom, default: :left_wheel],
      right_wheel: [type: :atom, default: :right_wheel]
    ]

  alias BB.Message
  alias BB.Message.Sensor.JointState
  alias BB.Robot.Units
  alias BB.StateMachine.Transition

  @impl BB.Controller
  def init(opts) do
    %{robot: robot} = Keyword.fetch!(opts, :bb)

    # Built from the topology rather than written out, so it's the same path the
    # supervisors gave the sensor: [:sensor, :world, :ground, :ground_contact, :lean, :lean_angle].
    {:ok, lean_joint_path} = BB.Robot.path_to(robot.robot(), :lean)
    lean_path = [:sensor | lean_joint_path] ++ [:lean_angle]

    {:ok, _} = BB.PubSub.subscribe(robot, [:state_machine])
    {:ok, _} = BB.PubSub.subscribe(robot, lean_path, message_types: [JointState])

    state = %{
      robot: robot,
      mode: BB.Robot.Runtime.state(robot),
      integral: 0.0,
      last: nil,
      last_command: 0.0
    }

    {:ok, configure(opts, state)}
  end

  # Fired when a param([...]) referenced in the DSL changes, e.g.
  # BB.Parameter.set(Goatbot.Robot, [:balance, :proportional_gain], 150.0)
  @impl BB.Controller
  def handle_options(opts, state), do: {:ok, configure(opts, state)}

  @impl BB.Controller
  def handle_call(:inspect, _from, state), do: {:reply, state, state}

  @impl BB.Controller
  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{to: to}}}, state) do
    # Start every attempt from a clean slate: no stale integral, no huge dt.
    {:noreply, %{state | mode: to, integral: 0.0, last: nil, last_command: 0.0}}
  end

  def handle_info(
        {:bb, _path, %Message{payload: %JointState{positions: [lean], velocities: [rate]}} = msg},
        %{mode: :balancing} = state
      ) do
    {:noreply, step(state, lean, rate, msg.monotonic_time)}
  end

  def handle_info(_message, state), do: {:noreply, state}

  defp step(state, lean, rate, now) do
    error = lean - state.setpoint

    if abs(error) > state.fall_angle do
      fall(state)
    else
      dt = if state.last, do: (now - state.last) / 1_000_000_000, else: 0.0

      integral =
        (state.integral + error * dt)
        |> max(-state.integral_limit)
        |> min(state.integral_limit)

      # The gyro's rate rather than a differentiated error: it's measured
      # directly, so it's far less noisy.
      command = state.kp * error + state.ki * integral + state.kd * rate

      # Leaning forward is positive, and the fix is to drive forward under it,
      # so both wheels take the command's sign as is.
      BB.Actuator.set_velocity(state.robot, state.left_wheel, command)
      BB.Actuator.set_velocity(state.robot, state.right_wheel, command)

      %{state | integral: integral, last: now, last_command: command}
    end
  end

  defp fall(state) do
    BB.Actuator.hold(state.robot, state.left_wheel)
    BB.Actuator.hold(state.robot, state.right_wheel)
    state.robot.fall()

    # Set locally rather than waiting for the transition to come back through
    # pubsub, or the samples arriving in the meantime would drive the wheels again.
    %{state | mode: :fallen, last: nil}
  end

  defp configure(opts, state) do
    Map.merge(state, %{
      setpoint: Units.to_radians(Keyword.fetch!(opts, :setpoint)),
      kp: Keyword.fetch!(opts, :kp),
      ki: Keyword.fetch!(opts, :ki),
      kd: Keyword.fetch!(opts, :kd),
      integral_limit: Keyword.fetch!(opts, :integral_limit),
      fall_angle: Units.to_radians(Keyword.fetch!(opts, :fall_angle)),
      left_wheel: Keyword.fetch!(opts, :left_wheel),
      right_wheel: Keyword.fetch!(opts, :right_wheel)
    })
  end
end
