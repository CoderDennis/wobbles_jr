import Config

config :logger, level: :info

config :wobbles_jr,
       WobblesJrWeb.Endpoint,
       force_ssl: [rewrite_on: [:x_forwarded_proto]],
       exclude: [hosts: ["localhost", "127.0.0.1"]]
