defmodule WobblesJr.Remote do
  @moduledoc """
  Drive commands from another Erlang node, such as the AtomVM badge's WobblesJr
  page.

  Registered as `WobblesJr.Remote`, so a peer sends to `{WobblesJr.Remote, node}`
  without needing a pid first:

      send({WobblesJr.Remote, :"wobbles_jr@wobbles_jr.local"}, {:drive, self(), throttle, turn})

  `throttle` and `turn` are both fractions from -1 to 1: positive is forwards
  and to the robot's left. The throttle goes to `BB.NSK.Drive.command/3`
  as it is, and the turn is scaled by the `[:yaw, :turn_rate]` parameter, so
  the robot keeps its own limits. Each command is answered with
  `{:wobbles_jr, state}`, the robot's state machine state, which tells the
  sender that the link is up. `{:hello, from}` gets the same answer without
  driving.

  `{:command, from, name}` runs `:arm`, `:stand` or `:disarm` on the robot,
  as `WobblesJr.IExHelpers` does, and answers `{:wobbles_jr_command, name, text}`
  once it finishes: `"ok"`, or a short reason. It runs in its own process,
  so drive commands keep flowing while the robot arms.

  There is no stop message and no watchdog here: the controllers already
  drop a drive request after `drive_timeout`, so a sender that goes quiet
  stops the robot.
  """

  use GenServer

  require Logger

  @robot WobblesJr.Robot
  @commands [:arm, :stand, :disarm]

  # Reasons are clipped to fit the badge's panel.
  @reason_bytes 24

  def start_link(opts), do: GenServer.start_link(__MODULE__, opts, name: __MODULE__)

  @doc """
  The throttle and turn rate, in rad/s, for a request of `throttle` and
  `turn` (both -1 to 1) at a full `turn_rate`. Anything else reads as zero.
  """
  @spec command(term, term, number) :: {float, float}
  def command(throttle, turn, turn_rate), do: {fraction(throttle), fraction(turn) * turn_rate}

  defp fraction(value) when is_number(value), do: value |> max(-1.0) |> min(1.0) |> Kernel.+(0.0)
  defp fraction(_value), do: 0.0

  @impl true
  def init(_opts), do: {:ok, %{peer: nil}}

  @impl true
  def handle_info({:drive, from, throttle, turn}, state) when is_pid(from) do
    {throttle, turn} = command(throttle, turn, turn_rate())
    safely(fn -> BB.NSK.Drive.command(@robot, throttle, turn) end)
    {:noreply, answer(from, state)}
  end

  def handle_info({:hello, from}, state) when is_pid(from), do: {:noreply, answer(from, state)}

  def handle_info({:command, from, name}, state) when is_pid(from) and name in @commands do
    Task.start(fn -> send(from, {:wobbles_jr_command, name, run(name)}) end)
    {:noreply, greet(node(from), state)}
  end

  def handle_info(_message, state), do: {:noreply, state}

  defp run(name) do
    case apply(@robot, name, []) do
      {:ok, pid} when is_pid(pid) -> describe(BB.Command.await(pid, 60_000))
      other -> describe(other)
    end
  catch
    kind, reason -> describe({:error, {kind, reason}})
  end

  @doc "A command's result as the badge shows it: `\"ok\"`, or a clipped reason."
  @spec describe(term) :: binary
  def describe({:ok, _result}), do: "ok"
  def describe({:ok, _result, _opts}), do: "ok"
  def describe({:error, %{__exception__: true} = error}), do: clip(Exception.message(error))
  def describe({:error, reason}), do: clip(inspect(reason))
  def describe(other), do: clip(inspect(other))

  defp clip(text) when byte_size(text) <= @reason_bytes, do: text
  defp clip(text), do: binary_part(text, 0, @reason_bytes)

  defp answer(from, state) do
    send(from, {:wobbles_jr, safely(fn -> BB.Robot.Runtime.state(@robot) end) || :unknown})
    greet(node(from), state)
  end

  # Logs a new driver once rather than every command.
  defp greet(peer, %{peer: peer} = state), do: state

  defp greet(peer, state) do
    Logger.info("Remote driver: #{peer}")
    %{state | peer: peer}
  end

  defp turn_rate, do: safely(fn -> BB.Parameter.get!(@robot, [:yaw, :turn_rate]) end) || 1.0

  defp safely(fun) do
    fun.()
  rescue
    _ -> nil
  catch
    :exit, _ -> nil
  end
end
