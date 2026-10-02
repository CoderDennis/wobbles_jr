defmodule WobblesJrWeb.Endpoint do
  use Phoenix.Endpoint, otp_app: :wobbles_jr

  @session_options [
    store: :cookie,
    key: "_wobbles_jr_key",
    signing_salt: "q96im8se",
    same_site: "Lax"
  ]

  socket("/live", Phoenix.LiveView.Socket,
    websocket: [connect_info: [session: @session_options]],
    longpoll: [connect_info: [session: @session_options]]
  )

  plug(Plug.Static, at: "/__bb_assets__", from: {:bb_liveview, "priv/static"}, gzip: false)

  plug(Plug.Static,
    at: "/",
    from: :wobbles_jr,
    gzip: not code_reloading?,
    only: WobblesJrWeb.static_paths(),
    raise_on_missing_only: code_reloading?
  )

  if code_reloading? do
    plug(Phoenix.CodeReloader)
  end

  plug(Plug.RequestId)
  plug(Plug.Telemetry, event_prefix: [:phoenix, :endpoint])

  plug(Plug.Parsers,
    parsers: [:urlencoded, :multipart, :json],
    pass: ["*/*"],
    json_decoder: Phoenix.json_library()
  )

  plug(Plug.MethodOverride)
  plug(Plug.Head)
  plug(Plug.Session, @session_options)
  plug(WobblesJrWeb.Router)
end
