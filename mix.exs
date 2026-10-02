defmodule WobblesJr.MixProject do
  use Mix.Project

  @app :wobbles_jr
  @version "0.1.0"
  @all_targets [:trellis]

  def project do
    [
      app: @app,
      version: @version,
      elixir: "~> 1.20",
      archives: [nerves_bootstrap: "~> 1.17"],
      listeners: listeners(Mix.target(), Mix.env()),
      start_permanent: Mix.env() == :prod,
      deps: deps(),
      releases: [{@app, release()}],
      aliases: aliases()
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger, :runtime_tools],
      mod: {WobblesJr.Application, []}
    ]
  end

  def cli do
    [preferred_targets: [run: :host, test: :host]]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:sourceror, "~> 1.7", runtime: false, only: [:dev, :test]},
      {:lazy_html, ">= 0.1.0", only: :test},
      {:tailwind, "~> 0.3", runtime: Mix.env() == :dev},
      {:esbuild, "~> 0.10", runtime: Mix.env() == :dev},
      {:phoenix_html, "~> 4.1"},
      {:phoenix_live_view, "~> 1.0"},
      {:telemetry_poller, "~> 1.0"},
      {:telemetry_metrics, "~> 1.0"},
      {:bandit, "~> 1.5"},
      {:phoenix, "~> 1.7"},
      {:vintage_net_wifi, "~> 0.12", targets: [:trellis]},
      {:video_interop, "~> 0.1.1"},
      {:emerge, "== 0.4.0-beta.1"},
      {:eink,
       [git: "https://github.com/emerge-elixir/eink.git", branch: "feat/imperative-gray2"]},
      {:hts221, [git: "https://harton.dev/james/hts221.git", branch: "main"]},
      {:circuits_i2c, "~> 2.1"},
      {:circuits_gpio, "~> 2.1"},
      {:sunxi,
       [
         github: "jimsynz/sunxi",
         branch: "fix/explain-missing-binary",
         targets: :host,
         runtime: false,
         override: true
       ]},
      {:nsk,
       [
         github: "jimsynz/nsk",
         branch: "fix/start-req-for-downloads",
         targets: :host,
         runtime: false
       ]},
      {:phx_install, "~> 0.1", only: [:dev, :test], runtime: false},
      {:bb_nsk, "~> 0.1"},
      {:igniter, "~> 0.6", only: [:dev, :test]},
      # Dependencies for all targets
      {:nerves, "~> 1.13", runtime: false},
      {:shoehorn, "~> 0.9.1"},
      {:ring_logger, "~> 0.11.0"},
      {:toolshed, "~> 0.5.0"},

      # Allow Nerves.Runtime on host to support development, testing and CI.
      # See config/host.exs for usage.
      {:nerves_runtime, "~> 0.13.12"},

      # Dependencies for all targets except :host
      {:nerves_pack, "~> 0.7.1", targets: @all_targets},
      # Dependencies for specific targets
      # NOTE: It's generally low risk and recommended to follow minor version
      # bumps to Nerves systems. Since these include Linux kernel and Erlang
      # version updates, please review their release notes in case
      # changes to your application are needed.
      {:nerves_system_trellis, "~> 0.5", runtime: false, targets: :trellis}
    ]
  end

  def release do
    [
      overwrite: true,
      # Erlang distribution is not started automatically.
      # See https://nerves-pack.hexdocs.pm/readme.html#erlang-distribution
      cookie: "#{@app}_cookie",
      include_erts: &Nerves.Release.erts/0,
      steps: [&Nerves.Release.init/1, :assemble],
      strip_beams: Mix.env() == :prod or [keep: ["Docs"]]
    ]
  end

  # Uncomment the following line if using Phoenix > 1.8.
  # defp listeners(:host, :dev), do: [Phoenix.CodeReloader]
  defp listeners(_, _), do: []

  defp aliases() do
    [
      firmware: ["assets.deploy", "bb_nsk.prune_nifs", "firmware"],
      test: ["bb_nsk.restore_nifs", "test"],
      run: ["bb_nsk.restore_nifs", "run"],
      "assets.setup": ["esbuild.install --if-missing", "tailwind.install --if-missing"],
      "assets.build": ["compile", "esbuild wobbles_jr", "tailwind wobbles_jr"],
      "assets.deploy": [
        "esbuild wobbles_jr --minify",
        "tailwind wobbles_jr --minify",
        "phx.digest"
      ],
      setup: ["deps.get", "assets.setup", "assets.build"]
    ]
  end
end
