import Config

if System.get_env("PHX_SERVER") do
  config :wobbles_jr, WobblesJrWeb.Endpoint, server: true
end

config :wobbles_jr, WobblesJrWeb.Endpoint,
  http: [port: String.to_integer(System.get_env("PORT", "4000"))]

if config_env() == :prod do
  secret_key_base =
    System.get_env("SECRET_KEY_BASE") ||
      raise """
      environment variable SECRET_KEY_BASE is missing.
      You can generate one by calling: mix phx.gen.secret
      """

  host = System.get_env("PHX_HOST") || "example.com"

  config :wobbles_jr, WobblesJrWeb.Endpoint,
    url: [host: host, port: 443, scheme: "https"],
    http: [ip: {0, 0, 0, 0, 0, 0, 0, 0}],
    secret_key_base: secret_key_base
end

# On the board the endpoint serves the drive pad on port 80 and starts
# itself — there is no `mix phx.server` on a device. `check_origin` is off
# because the robot is reached by IP, on its own access point as often as
# not, so there is no origin to check against.
if config_target() != :host do
  config :wobbles_jr, WobblesJrWeb.Endpoint,
    http: [ip: {0, 0, 0, 0, 0, 0, 0, 0}, port: 80],
    server: true,
    check_origin: false
end
