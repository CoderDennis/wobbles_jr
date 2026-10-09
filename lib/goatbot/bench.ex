defmodule Goatbot.Bench do
  @moduledoc """
  Benchmarks for comparing BEAM builds on the robot, such as the interpreter
  against the ARM32 JIT. Compiled into the firmware, so the measuring code is
  the same on both builds; an `ssh` eval script would be interpreted by
  `erl_eval`, which the JIT speeds up too.

      Goatbot.Bench.run(label: "jit")       # everything, saved to /data/bench
      Goatbot.Bench.compare("emu", "jit")   # side by side

  ## What it measures

    1. **Micro** (`micro/1`): CPU time per call of the work the robot does per
       sample — the Mahony step, the lean maths, building the IMU and joint
       state messages, a PubSub publish, the velocity and yaw ticks — plus one
       dashboard render, and a generic float and map workload for reference.
       Pure computation: it shows how much faster the code is, nothing else.

    2. **Pipeline** (`pipeline/2`): the running robot, read from the timestamps
       each stage stamps on its message, so the benchmark's own scheduling
       doesn't enter into it. The IMU's sample interval (is it on time at
       200 Hz?), the latency from IMU sample to orientation to lean to wheel
       command, scheduler utilisation, and microstate accounting (how scheduler
       time splits between emulator, GC, NIFs, ports and sleep). While the robot
       is balancing it adds the robot's own result: lean-rate jitter, overall and
       above 5 Hz (where the 7 Hz wobble lives), and the largest rate.

    3. **Scenarios** (`run/1`): the pipeline under fixed extra load:
       `:idle`, `:dashboard` (the real dashboard view rendered headlessly 4
       times a second, the known trigger for the wobble) and `{:load, n}`
       (n Mahony steps per second of synthetic work, split over one worker per
       scheduler). Load is a fixed amount of *work*, not CPU time, so a faster
       runtime shows up as more headroom: the same work costs less CPU and
       delays the balance loop less.

  The I2C read inside the IMU's poll is a NIF and the same on both builds.

  ## Running a fair comparison

  Same firmware apart from the runtime, same parameters, same floor, and a
  similar battery charge. Run the CPU-only parts (`micro/1`, and `run/1` with
  the robot disarmed) for the cleanest numbers; run `run/1` again while
  balancing for the robot's result, and go A, B, A to see how much the battery
  or the floor moved between runs. Heavier load levels may make it fall over:
  that is the measurement, but hold on to it.
  """

  alias BB.Estimator.Ahrs.Mahony
  alias BB.Math.{Quaternion, Vec3}
  alias BB.Message
  alias BB.Message.Actuator.Command.Velocity
  alias BB.Message.Sensor.{Imu, JointState}
  alias BB.NSK.Sensor.Lean

  @robot Goatbot.Robot
  @dir "/data/bench"
  @processes [:imu, :orientation, :lean_angle, :heading, :pid_balancer, :velocity_hold, :yaw_hold]

  @doc """
  Runs everything, prints a report and saves the results.

  Options: `:label` (the file name, default the emulator flavour), `:seconds`
  per scenario (20), `:scenarios` (default `[:idle, :dashboard, {:load,
  2_000}, {:load, 5_000}, {:load, 10_000}]`) and `:micro` (true).
  """
  def run(opts \\ []) do
    info = info()
    label = Keyword.get(opts, :label, to_string(info.emu_flavor))
    seconds = Keyword.get(opts, :seconds, 20)

    scenarios =
      Keyword.get(opts, :scenarios, [
        :idle,
        :dashboard,
        {:load, 2_000},
        {:load, 5_000},
        {:load, 10_000}
      ])

    print_info(info)
    micro = if Keyword.get(opts, :micro, true), do: micro(), else: %{}

    pipeline =
      for scenario <- scenarios do
        result = with_scenario(scenario, fn -> pipeline(seconds) end)
        print_pipeline(scenario, result)
        {scenario, result}
      end

    results = %{label: label, info: info, micro: micro, pipeline: pipeline}
    save(label, results)
    results
  end

  @doc "The runtime and the board it is running on."
  def info do
    %{
      emu_flavor: :erlang.system_info(:emu_flavor),
      emu_type: :erlang.system_info(:emu_type),
      otp_release: to_string(:erlang.system_info(:otp_release)),
      system_version: :erlang.system_info(:system_version) |> to_string() |> String.trim(),
      schedulers: :erlang.system_info(:schedulers_online),
      governor: read_sys("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
      cpu_khz: read_sys("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"),
      loadavg: read_sys("/proc/loadavg"),
      robot_state: robot_state(),
      at: DateTime.utc_now()
    }
  end

  # ----------------------------------------------------------------------------
  # Micro
  # ----------------------------------------------------------------------------

  @doc """
  Microseconds per call of each piece of per-sample work: the median of
  `reps` runs of `n` calls each, after a warm-up.
  """
  def micro(opts \\ []) do
    n = Keyword.get(opts, :n, 2_000)
    reps = Keyword.get(opts, :reps, 7)
    samples = synthetic_imu(n)
    body_from_imu = Lean.body_from_imu(@robot, :base_link, :imu_link)
    mahony = %Mahony{q: Quaternion.identity(), kp: 0.2, ki: 0.005}

    cases = [
      mahony_step: fn ->
        Enum.reduce(samples, mahony, fn {g, a}, s -> Mahony.step(s, g, a, 0.005) end)
      end,
      lean: fn ->
        for {{gx, gy, gz}, _} <- samples do
          q = Quaternion.new(0.99, 0.01 * gx, 0.01 * gy, 0.01 * gz)
          {Lean.lean(q, body_from_imu), Lean.lean_rate(Vec3.new(gx, gy, gz), body_from_imu)}
        end
      end,
      imu_message: fn ->
        for {{gx, gy, gz}, {ax, ay, az}} <- samples do
          {:ok, m} =
            Imu.new(:imu_link,
              orientation: Quaternion.identity(),
              angular_velocity: Vec3.new(gx, gy, gz),
              linear_acceleration: Vec3.new(ax, ay, az)
            )

          m
        end
      end,
      joint_state_message: fn ->
        for {{gx, _, _}, _} <- samples do
          {:ok, m} =
            JointState.new(:lean_angle, names: [:lean], positions: [gx], velocities: [gx])

          m
        end
      end,
      pubsub_publish: fn -> publish_and_drain(samples) end,
      velocity_tick:
        tick_case(:velocity_hold, n, fn s -> Goatbot.VelocityController.tick(s, 0.02) end),
      yaw_tick: tick_case(:yaw_hold, n, fn s -> Goatbot.YawController.tick(s, 0.5, 0.02) end),
      reference_float: fn ->
        Enum.reduce(samples, 0.0, fn {{gx, gy, gz}, _}, acc ->
          acc + :math.sin(gx) * :math.cos(gy) + :math.sqrt(abs(gz))
        end)
      end,
      reference_map: fn ->
        Enum.reduce(samples, %{a: 0.0, b: 0, c: nil}, fn {{gx, _, _}, _}, m ->
          %{m | a: m.a + gx, b: m.b + 1, c: {gx}}
        end)
      end
    ]

    IO.puts("\nMicro: us per call, median of #{reps} x #{n}")

    results =
      for {name, fun} <- cases, fun != nil, into: %{} do
        us = time_per_call(fun, n, reps)
        IO.puts("  #{String.pad_trailing(to_string(name), 22)} #{fmt(us, 2)}")
        {name, us}
      end

    render = dashboard_render(Keyword.get(opts, :renders, 20))
    IO.puts("  #{String.pad_trailing("dashboard_render", 22)} #{fmt(render, 0)}")
    Map.put(results, :dashboard_render, render)
  end

  defp time_per_call(fun, n, reps) do
    fun.()

    times =
      for _ <- 1..reps do
        :erlang.garbage_collect()
        {us, _} = :timer.tc(fun)
        us / n
      end

    median(times)
  end

  # Ticks a controller's live state, forced into :balancing, so the benchmark
  # runs the same map the controller does. Skipped if it isn't running.
  defp tick_case(name, n, tick) do
    case live_state(name) do
      nil ->
        nil

      state ->
        state = %{state | mode: :balancing}
        fn -> Enum.reduce(1..n, state, fn _, s -> tick.(s) end) end
    end
  end

  defp live_state(name) do
    BB.Process.call(@robot, name, :inspect)
  catch
    :exit, _ -> nil
  end

  defp publish_and_drain(samples) do
    path = [:bench, :publish]
    {:ok, _} = BB.PubSub.subscribe(@robot, path)
    {:ok, m} = JointState.new(:bench, names: [:x], positions: [0.0], velocities: [0.0])

    for _ <- samples do
      BB.PubSub.publish(@robot, path, m)

      receive do
        {:bb, ^path, _} -> :ok
      end
    end

    BB.PubSub.unsubscribe(@robot, path)
  end

  defp dashboard_render(n) do
    session = Breeze.Test.start!(Goatbot.Dashboard, size: {110, 36})

    try do
      Breeze.Test.render!(session)
      {us, _} = :timer.tc(fn -> for _ <- 1..n, do: Breeze.Test.render!(session) end)
      us / n
    after
      Breeze.Test.stop(session)
    end
  end

  # A 2 Hz wobble with noise, at 200 Hz: the shape, not the values, matters.
  defp synthetic_imu(n) do
    :rand.seed(:exsss, {1, 2, 3})

    for i <- 1..n do
      t = i / 200
      w = 0.3 * :math.sin(2 * :math.pi() * 2 * t)

      {{w + noise(0.05), noise(0.01), noise(0.01)},
       {noise(0.2), -9.7 + noise(0.2), 0.5 + :math.sin(w) + noise(0.2)}}
    end
  end

  defp noise(scale), do: (:rand.uniform() - 0.5) * 2 * scale

  # ----------------------------------------------------------------------------
  # Pipeline
  # ----------------------------------------------------------------------------

  @doc """
  Watches the running robot for `seconds` and summarises its timing, CPU and,
  if it is balancing, its lean-rate jitter.
  """
  def pipeline(seconds \\ 20) do
    robot = @robot.robot()
    {:ok, lean_joint} = BB.Robot.path_to(robot, :lean)
    {:ok, imu_link} = BB.Robot.path_to(robot, :imu_link)
    {:ok, wheel} = BB.Robot.actuator_path(robot, :left_wheel)
    imu_path = [:sensor | imu_link] ++ [:imu]
    orientation_path = imu_path ++ [:orientation]
    lean_path = [:sensor | lean_joint] ++ [:lean_angle]
    command_path = [:actuator | wheel]

    paths = [imu: imu_path, orientation: orientation_path, lean: lean_path, command: command_path]
    stage = Map.new(paths, fn {name, path} -> {path, name} end)
    drain(stage)
    for {_, path} <- paths, do: {:ok, _} = BB.PubSub.subscribe(@robot, path)
    started = System.monotonic_time(:nanosecond)

    :erlang.system_flag(:scheduler_wall_time, true)
    state_before = robot_state()
    reductions_before = reductions()
    scheduler_before = :scheduler.sample_all()
    :msacc.start()

    deadline = System.monotonic_time(:millisecond) + seconds * 1000
    # Only what was stamped inside the window: anything older would be counted
    # as a gap across whatever ran between two calls.
    rows = for {_, t, _} = row <- collect(stage, deadline, []), t >= started, do: row

    :msacc.stop()
    msacc = :msacc.stats()
    scheduler_after = :scheduler.sample_all()
    reductions_after = reductions()
    for {_, path} <- paths, do: BB.PubSub.unsubscribe(@robot, path)
    drain(stage)

    times = fn name -> for {^name, t, _} <- rows, do: t end
    imu = times.(:imu)
    orientation = times.(:orientation)
    lean = times.(:lean)
    command = times.(:command)
    intervals = Enum.zip_with(imu, Enum.drop(imu, 1), &((&2 - &1) / 1.0e6))
    rates = for {:lean, _, rate} <- rows, do: rate * 180 / :math.pi()

    %{
      seconds: seconds,
      robot_state_before: state_before,
      robot_state: robot_state(),
      imu_hz: length(imu) / seconds,
      imu_interval: stats(intervals) |> Map.put(:late_pct, pct_over(intervals, 7.5)),
      imu_to_orientation: stats(latencies(imu, orientation)),
      orientation_to_lean: stats(latencies(orientation, lean)),
      lean_to_command: stats(latencies(lean, command)),
      imu_to_command: stats(latencies(imu, command)),
      scheduler_util: scheduler_util(scheduler_before, scheduler_after),
      msacc: msacc_summary(msacc),
      reductions_per_s: reductions_per_s(reductions_before, reductions_after, seconds),
      lean_rate: jitter(rates)
    }
  end

  # Throws away messages from these paths still in the mailbox.
  defp drain(stage) do
    receive do
      {:bb, path, _} when is_map_key(stage, path) -> drain(stage)
    after
      0 -> :ok
    end
  end

  defp collect(stage, deadline, acc) do
    timeout = deadline - System.monotonic_time(:millisecond)

    if timeout <= 0 do
      Enum.reverse(acc)
    else
      receive do
        {:bb, path, %Message{payload: payload, monotonic_time: t}} ->
          case Map.fetch(stage, path) do
            {:ok, :lean} ->
              collect(stage, deadline, [{:lean, t, hd(payload.velocities)} | acc])

            {:ok, :command} when is_struct(payload, Velocity) ->
              collect(stage, deadline, [{:command, t, nil} | acc])

            {:ok, :command} ->
              collect(stage, deadline, acc)

            {:ok, name} ->
              collect(stage, deadline, [{name, t, nil} | acc])

            :error ->
              collect(stage, deadline, acc)
          end
      after
        timeout -> Enum.reverse(acc)
      end
    end
  end

  # For each upstream time, how long until the first downstream message at or
  # after it, as long as that comes before the next upstream one (otherwise the
  # sample was dropped downstream and doesn't count).
  defp latencies([], _), do: []
  defp latencies(_, []), do: []

  defp latencies([a | as], [b | _] = bs) when b < a, do: latencies([a | as], tl(bs))

  defp latencies([a | as], [b | bs]) do
    next = List.first(as)

    if next == nil or b < next,
      do: [(b - a) / 1.0e6 | latencies(as, bs)],
      else: latencies(as, [b | bs])
  end

  defp jitter([]), do: nil

  defp jitter(rates) do
    # Above ~5 Hz: the rate minus its 0.2 s moving average.
    fast =
      rates
      |> Enum.chunk_every(40, 1, :discard)
      |> Enum.map(fn chunk -> List.last(chunk) - Enum.sum(chunk) / 40 end)

    %{rms: rms(rates), fast_rms: rms(fast), max: rates |> Enum.map(&abs/1) |> Enum.max()}
  end

  defp scheduler_util(before, after_) do
    # The normal schedulers only: the dirty ones sit idle and would halve it.
    normal = for {:normal, _id, u, _} <- :scheduler.utilization(before, after_), do: u * 100
    if normal == [], do: nil, else: Enum.sum(normal) / length(normal)
  end

  # Where scheduler time went, as a percentage of all scheduler threads' time.
  defp msacc_summary(stats) do
    counters =
      for %{type: :scheduler, counters: c} <- stats, reduce: %{} do
        acc -> Map.merge(acc, c, fn _, x, y -> x + y end)
      end

    total = counters |> Map.values() |> Enum.sum()
    if total > 0, do: Map.new(counters, fn {k, v} -> {k, v * 100 / total} end), else: %{}
  end

  defp reductions do
    for name <- @processes, pid = BB.Process.whereis(@robot, name), is_pid(pid), into: %{} do
      {name, elem(Process.info(pid, :reductions) || {:reductions, 0}, 1)}
    end
  end

  defp reductions_per_s(before, after_, seconds) do
    for {name, r} <- after_, into: %{}, do: {name, (r - Map.get(before, name, r)) / seconds}
  end

  # ----------------------------------------------------------------------------
  # Scenarios
  # ----------------------------------------------------------------------------

  @doc "Runs `fun` with the extra load `scenario` describes, then removes it."
  def with_scenario(:idle, fun), do: fun.()

  def with_scenario(:dashboard, fun) do
    session = Breeze.Test.start!(Goatbot.Dashboard, size: {110, 36})
    renderer = spawn_link(fn -> render_loop(session) end)

    try do
      fun.()
    after
      Process.unlink(renderer)
      Process.exit(renderer, :kill)
      Breeze.Test.stop(session)
    end
  end

  def with_scenario({:load, steps_per_s}, fun) do
    workers = :erlang.system_info(:schedulers_online)
    per_tick = max(div(steps_per_s, 100 * workers), 1)
    pids = for _ <- 1..workers, do: spawn(fn -> load_loop(per_tick, synthetic_imu(per_tick)) end)

    try do
      fun.()
    after
      for pid <- pids, do: Process.exit(pid, :kill)
    end
  end

  defp render_loop(session) do
    Breeze.Test.render!(session)
    Process.sleep(250)
    render_loop(session)
  end

  # `per_tick` Mahony steps every 10 ms: a fixed amount of work, whatever it
  # costs.
  defp load_loop(per_tick, samples) do
    started = System.monotonic_time(:millisecond)
    mahony = %Mahony{q: Quaternion.identity(), kp: 0.2, ki: 0.005}
    Enum.reduce(samples, mahony, fn {g, a}, s -> Mahony.step(s, g, a, 0.005) end)
    elapsed = System.monotonic_time(:millisecond) - started
    if elapsed < 10, do: Process.sleep(10 - elapsed)
    load_loop(per_tick, samples)
  end

  # ----------------------------------------------------------------------------
  # Saving and comparing
  # ----------------------------------------------------------------------------

  defp save(label, results) do
    path = Path.join(dir(), "#{label}.etf")
    File.mkdir_p!(dir())
    File.write!(path, :erlang.term_to_binary(results))
    IO.puts("\nSaved #{path}")
  end

  @doc "Loads saved results by label."
  def load(label),
    do: Path.join(dir(), "#{label}.etf") |> File.read!() |> :erlang.binary_to_term()

  @doc "Prints two saved runs side by side, with `b / a` for each number."
  def compare(a, b) do
    a = load(a)
    b = load(b)
    IO.puts("#{pad("", 34)} #{pad(a.label, 12)} #{pad(b.label, 12)} b/a")
    IO.puts("runtime #{a.info.emu_flavor} vs #{b.info.emu_flavor}")

    IO.puts("Micro, us per call")

    for {name, x} <- Enum.sort(a.micro), y = b.micro[name], is_number(x) and is_number(y) do
      row("  #{name}", x, y, 2)
    end

    for {scenario, x} <- a.pipeline, y = List.keyfind(b.pipeline, scenario, 0) do
      y = elem(y, 1)

      IO.puts(
        "#{inspect(scenario)} (#{x.robot_state_before}->#{x.robot_state} / #{y.robot_state_before}->#{y.robot_state})"
      )

      row("  scheduler util %", x.scheduler_util, y.scheduler_util, 1)
      row("  msacc emulator %", x.msacc[:emulator], y.msacc[:emulator], 1)
      row("  msacc gc %", x.msacc[:gc], y.msacc[:gc], 1)
      row("  imu interval p99 ms", x.imu_interval[:p99], y.imu_interval[:p99], 2)
      row("  imu late % (>7.5 ms)", x.imu_interval[:late_pct], y.imu_interval[:late_pct], 2)
      row("  imu->lean p50 ms", sum_p(x, :p50), sum_p(y, :p50), 2)
      row("  imu->command p50 ms", x.imu_to_command[:p50], y.imu_to_command[:p50], 2)
      row("  imu->command p99 ms", x.imu_to_command[:p99], y.imu_to_command[:p99], 2)
      row("  imu->command max ms", x.imu_to_command[:max], y.imu_to_command[:max], 2)

      row(
        "  lean rate fast rms deg/s",
        get_in(x, [:lean_rate, :fast_rms]),
        get_in(y, [:lean_rate, :fast_rms]),
        1
      )

      row(
        "  lean rate max deg/s",
        get_in(x, [:lean_rate, :max]),
        get_in(y, [:lean_rate, :max]),
        0
      )
    end

    :ok
  end

  defp sum_p(r, p) do
    with x when is_number(x) <- r.imu_to_orientation[p],
         y when is_number(y) <- r.orientation_to_lean[p],
         do: x + y
  end

  defp row(name, x, y, digits) do
    ratio = if is_number(x) and is_number(y) and x != 0, do: fmt(y / x, 2), else: "-"
    IO.puts("#{pad(name, 34)} #{pad(fmt(x, digits), 12)} #{pad(fmt(y, digits), 12)} #{ratio}")
  end

  # ----------------------------------------------------------------------------
  # Printing and helpers
  # ----------------------------------------------------------------------------

  defp print_info(info) do
    IO.puts("""
    #{info.system_version}
    emu_flavor #{info.emu_flavor}, #{info.schedulers} schedulers, governor #{info.governor}, #{info.cpu_khz} kHz
    loadavg #{info.loadavg}, robot #{info.robot_state}\
    """)
  end

  defp print_pipeline(scenario, r) do
    IO.puts("""

    #{inspect(scenario)}: robot #{r.robot_state_before} -> #{r.robot_state}, IMU #{fmt(r.imu_hz, 1)} Hz, scheduler #{fmt(r.scheduler_util, 1)}%, \
    emulator #{fmt(r.msacc[:emulator], 1)}% gc #{fmt(r.msacc[:gc], 1)}% sleep #{fmt(r.msacc[:sleep], 1)}%
      imu interval   #{fmt_stats(r.imu_interval)}  late #{fmt(r.imu_interval[:late_pct], 2)}%
      imu->orient     #{fmt_stats(r.imu_to_orientation)}
      orient->lean    #{fmt_stats(r.orientation_to_lean)}
      lean->command   #{fmt_stats(r.lean_to_command)}
      imu->command    #{fmt_stats(r.imu_to_command)}\
    """)

    if r.lean_rate do
      IO.puts(
        "  lean rate rms #{fmt(r.lean_rate.rms, 1)} deg/s, fast #{fmt(r.lean_rate.fast_rms, 1)} deg/s, max #{fmt(r.lean_rate.max, 0)} deg/s"
      )
    end
  end

  defp fmt_stats(%{n: 0}), do: "-"

  defp fmt_stats(s),
    do:
      "p50 #{fmt(s.p50, 2)}  p95 #{fmt(s.p95, 2)}  p99 #{fmt(s.p99, 2)}  max #{fmt(s.max, 2)} ms"

  defp stats([]), do: %{n: 0}

  defp stats(xs) do
    sorted = Enum.sort(xs)
    n = length(sorted)
    at = fn q -> Enum.at(sorted, min(trunc(n * q), n - 1)) end
    %{n: n, p50: at.(0.5), p95: at.(0.95), p99: at.(0.99), max: List.last(sorted)}
  end

  defp pct_over([], _), do: 0.0
  defp pct_over(xs, limit), do: Enum.count(xs, &(&1 > limit)) * 100 / length(xs)

  defp median(xs), do: xs |> Enum.sort() |> Enum.at(div(length(xs), 2))

  defp rms([]), do: 0.0
  defp rms(xs), do: :math.sqrt(Enum.sum(Enum.map(xs, &(&1 * &1))) / length(xs))

  defp fmt(x, digits) when is_float(x), do: :erlang.float_to_binary(x, decimals: digits)
  defp fmt(x, _) when is_integer(x), do: Integer.to_string(x)
  defp fmt(_, _), do: "-"

  defp pad(s, n), do: String.pad_trailing(to_string(s), n)

  defp robot_state do
    BB.Robot.Runtime.state(@robot)
  catch
    :exit, _ -> :not_running
  end

  defp read_sys(path) do
    case File.read(path) do
      {:ok, s} -> String.trim(s)
      _ -> "-"
    end
  end

  defp dir do
    if File.dir?("/data"), do: @dir, else: Path.join(System.tmp_dir!(), "goatbot_bench")
  end
end
