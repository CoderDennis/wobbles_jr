import Config

config :phoenix, plug_init_mode: :runtime
config :logger, level: :warning

config :wobbles_jr,
       WobblesJrWeb.Endpoint,
       http: [ip: {127, 0, 0, 1}, port: 4002],
       secret_key_base: "KQTaPsCMzll9WPSkrQ/OtYe0n/838QMMEuLItQ+HX+jJDQuH6XXS1m0UQIKHD8Ha",
       server: false
