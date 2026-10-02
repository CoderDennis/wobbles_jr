defmodule WobblesJrWeb.Router do
  use WobblesJrWeb, :router

  pipeline :browser do
    plug(:accepts, ["html"])
    plug(:fetch_session)
    plug(:fetch_live_flash)
    plug(:put_root_layout, html: {WobblesJrWeb.Layouts, :root})
    plug(:protect_from_forgery)
    plug(:put_secure_browser_headers)
  end

  pipeline :api do
    plug(:accepts, ["json"])
  end

  scope "/", WobblesJrWeb do
    pipe_through([:browser])
    live("/", WifiLive)
    live("/drive", DriveLive)
  end

  scope "/" do
    pipe_through([:browser])
    import BB.LiveView.Router
    bb_dashboard("/dashboard", robot: WobblesJr.Robot)
  end

  # No pipeline: MCP clients speak JSON and don't carry a CSRF token.
  scope "/" do
    import BB.MCP.Router
    bb_mcp("/mcp")
  end

  scope "/api", WobblesJrWeb do
    pipe_through(:api)
  end
end
