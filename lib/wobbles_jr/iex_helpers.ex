defmodule WobblesJr.IExHelpers do
  @moduledoc """
  Helpers for poking at the running robot from an IEx session.

  Imported by `rootfs_overlay/etc/iex.exs`, so they're there whenever you SSH
  in. `goat/0` lists them.
  """

  @robot WobblesJr.Robot
  @balancer :pid_balancer
  @velocity :velocity_hold
  @gains [
    p: [:balance, :proportional_gain],
    i: [:balance, :integral_gain],
    d: [:balance, :derivative_gain],
    v: [:velocity, :velocity_damping]
  ]

  def goat do
    IO.puts("""
    WobblesJr helpers
      dash()                      full-screen dashboard (q to quit)
      status()                    state, lean, controller, gains
      go()                        arm and stand: hold it upright first
      arm() | stand() | disarm()
      watch(hz \\\\ 10) | unwatch()  live lean, error, command, velocity estimate, offset
      gains()                     balance gains and the velocity damping
      gains(p: 100, i: 400, d: 1.5, v: 0.0043)
                                  set any of p i d v together
      params(prefix \\\\ [])        every parameter, e.g. params([:balance])
      set(:name, value)           one parameter by name or path; plain numbers
                                  work for unit params (degrees, hertz, ...)
      listen(path, n \\\\ 3)        print the next n PubSub messages under path,
                                  e.g. listen([:state_machine]) or listen([:sensor])
      procs()                     every process the robot has registered
      start_robot()               start the robot if the app didn't
    """)
  end

  ## State

  @doc "Opens the full-screen `WobblesJr.Dashboard`; `q` hands the shell back."
  def dash, do: WobblesJr.Dashboard.run()

  def status do
    with :ok <- running() do
      lean = Map.get(BB.Robot.Runtime.configurations(@robot), :lean)

      IO.puts("state     #{BB.Robot.Runtime.state(@robot)} (safety #{BB.Safety.state(@robot)})")
      IO.puts("lean      #{deg(lean)}")

      case balancer() do
        nil ->
          IO.puts("balancer  not running")

        c ->
          IO.puts(
            "setpoint  #{deg(c.setpoint)}   target #{deg(c.target)}   error #{deg(error(lean, c))}"
          )

          IO.puts("command   #{num(c.last_command)} rad/s   integral #{num(c.integral, 7, 4)}")
      end

      case velocity() do
        nil ->
          IO.puts("velocity  controller not running")

        v ->
          IO.puts(
            "velocity  #{num(v.velocity)}   average cmd #{num(v.average)} rad/s   offset #{deg(v.offset)}"
          )
      end

      gains()
    end
  end

  def watch(hz \\ 10) do
    unwatch()
    shell = self()

    pid =
      spawn(fn ->
        # Stops with the SSH session rather than printing into a dead one.
        ref = Process.monitor(shell)
        watch_loop(ref, div(1000, hz))
      end)

    Process.register(pid, :wobbles_jr_watch)
    :ok
  end

  def unwatch do
    case Process.whereis(:wobbles_jr_watch) do
      nil ->
        :ok

      pid ->
        ref = Process.monitor(pid)
        Process.exit(pid, :kill)

        receive do
          {:DOWN, ^ref, _, _, _} -> :ok
        end
    end
  end

  defp watch_loop(ref, ms) do
    receive do
      {:DOWN, ^ref, _, _, _} -> :ok
    after
      ms ->
        IO.puts(watch_line())
        watch_loop(ref, ms)
    end
  end

  defp watch_line do
    with true <- running?(),
         %{} = c <- balancer() do
      lean = Map.get(BB.Robot.Runtime.configurations(@robot), :lean)

      v = velocity() || %{velocity: nil, offset: nil}

      "lean #{deg(lean)}  error #{deg(error(lean, c))}  cmd #{num(c.last_command, 7)}" <>
        "  vel #{num(v.velocity)}  offset #{deg(v.offset)}  #{c.mode}"
    else
      _ -> "(robot or balancer not running)"
    end
  end

  ## Commands

  def arm, do: command(:arm)
  def stand, do: command(:stand)
  def disarm, do: command(:disarm)

  def go do
    with :ok <- running() do
      if BB.Safety.armed?(@robot), do: stand(), else: then_stand(arm())
    end
  end

  defp then_stand({:ok, _} = _armed), do: stand()
  defp then_stand({:ok, _, _} = _armed), do: stand()
  defp then_stand(error), do: error

  defp command(name) do
    with :ok <- running(),
         {:ok, pid} <- apply(@robot, name, []) do
      BB.Command.await(pid)
    end
  end

  ## Parameters

  def gains(new \\ []) do
    with :ok <- running(),
         :ok <- set_gains(new) do
      @gains
      |> Enum.map_join("  ", fn {key, path} ->
        "#{key}=#{show(BB.Parameter.get!(@robot, path))}"
      end)
      |> IO.puts()
    end
  end

  defp set_gains([]), do: :ok

  defp set_gains(new) do
    case Enum.reject(Keyword.keys(new), &Keyword.has_key?(@gains, &1)) do
      [] ->
        BB.Parameter.set_many(@robot, for({k, v} <- new, do: {@gains[k], v / 1}))

      unknown ->
        {:error, {:unknown_gains, unknown, known: Keyword.keys(@gains)}}
    end
  end

  def params(prefix \\ []) do
    with :ok <- running() do
      @robot
      |> BB.Parameter.list(prefix: prefix)
      |> Enum.sort()
      |> Enum.each(fn {path, meta} ->
        IO.puts(String.pad_trailing(Enum.join(path, "."), 32) <> show(meta.value))
      end)
    end
  end

  # Parameter names are unique across groups, so a bare name is enough.
  def set(name, value) when is_atom(name) do
    with :ok <- running() do
      case Enum.filter(BB.Parameter.list(@robot), fn {path, _} -> List.last(path) == name end) do
        [{path, meta}] -> do_set(path, meta, value)
        [] -> {:error, {:no_parameter, name}}
        many -> {:error, {:ambiguous, Enum.map(many, &elem(&1, 0))}}
      end
    end
  end

  def set(path, value) when is_list(path) do
    with :ok <- running() do
      case List.keyfind(BB.Parameter.list(@robot, prefix: path), path, 0) do
        {^path, meta} -> do_set(path, meta, value)
        nil -> {:error, {:no_parameter, path}}
      end
    end
  end

  defp do_set(path, meta, value) do
    with :ok <- BB.Parameter.set(@robot, path, coerce(meta.type, value)) do
      IO.puts("#{Enum.join(path, ".")} = #{show(BB.Parameter.get!(@robot, path))}")
    end
  end

  # `{:unit, :degree_per_second}` wants `Localize.Unit.new!(n, "degree-per-second")`.
  defp coerce({:unit, unit}, value) when is_number(value) do
    Localize.Unit.new!(value, unit |> Atom.to_string() |> String.replace("_", "-"))
  end

  defp coerce(:float, value) when is_integer(value), do: value / 1
  defp coerce(_type, value), do: value

  ## Plumbing

  def listen(path \\ [], n \\ 3) do
    with :ok <- running() do
      {:ok, _} = BB.PubSub.subscribe(@robot, path)

      try do
        Enum.reduce_while(1..n, :ok, fn _, acc ->
          receive do
            {:bb, at, %BB.Message{payload: payload}} ->
              IO.puts("#{inspect(at)}\n  #{inspect(payload)}")
              {:cont, acc}
          after
            2000 ->
              IO.puts("(nothing for 2s)")
              {:halt, acc}
          end
        end)
      after
        BB.PubSub.unsubscribe(@robot, path)
        drain()
      end
    end
  end

  # A 200 Hz topic keeps arriving until the unsubscribe lands; keep the shell's
  # mailbox clean.
  defp drain do
    receive do
      {:bb, _, _} -> drain()
    after
      0 -> :ok
    end
  end

  def procs do
    with :ok <- running() do
      @robot
      |> BB.Process.registry_name()
      |> Registry.select([{{:"$1", :"$2", :_}, [], [{{:"$1", :"$2"}}]}])
      |> Enum.sort_by(&inspect(elem(&1, 0)))
      |> Enum.each(fn {name, pid} ->
        IO.puts(String.pad_trailing(inspect(name), 40) <> inspect(pid))
      end)
    end
  end

  def start_robot(opts \\ []) do
    if running?() do
      {:ok, Process.whereis(@robot)}
    else
      case Supervisor.start_child(WobblesJr.Supervisor, {@robot, opts}) do
        {:error, :already_present} -> Supervisor.restart_child(WobblesJr.Supervisor, @robot)
        other -> other
      end
    end
  end

  ## Formatting

  defp running?, do: Process.whereis(@robot) != nil

  defp running do
    if running?() do
      :ok
    else
      IO.puts("WobblesJr.Robot isn't running. start_robot() starts it.")
      {:error, :robot_not_running}
    end
  end

  defp balancer do
    BB.Process.call(@robot, @balancer, :inspect)
  catch
    :exit, _ -> nil
  end

  defp velocity do
    BB.Process.call(@robot, @velocity, :inspect)
  catch
    :exit, _ -> nil
  end

  # Against the target the balancer is actually chasing, offset included.
  defp error(nil, _balancer), do: nil
  defp error(_lean, %{target: nil}), do: nil
  defp error(lean, balancer), do: lean - balancer.target

  defp deg(nil), do: "     -  "
  defp deg(radians), do: num(radians * 180 / :math.pi(), 6, 2) <> "°"

  defp num(value, width \\ 6, decimals \\ 3)
  defp num(nil, width, _decimals), do: String.pad_leading("-", width)

  defp num(value, width, decimals) do
    value
    |> Kernel./(1)
    |> :erlang.float_to_binary(decimals: decimals)
    |> String.pad_leading(width)
  end

  defp show(%Localize.Unit{value: value, name: name}), do: "#{value} #{name}"
  defp show(value) when is_float(value), do: :erlang.float_to_binary(value, [:short])
  defp show(value), do: inspect(value)
end
