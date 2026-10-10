defmodule WobblesJr.Distribution do
  @moduledoc """
  Runs the robot's Erlang node as `wobbles_jr@<wlan0 address>`, so the badge can
  reach `WobblesJr.Remote` by IP.

  The node starts once wifi has an IPv4 address and is renamed if the address
  changes. It is named for the address rather than for `wobbles_jr.local`
  because AtomVM checks the name it reaches against the one it asked for, and
  the badge can't resolve `.local` names yet: `mdns_lite` leaves the question
  out of legacy unicast answers, which lwIP's resolver discards.
  """

  use GenServer

  require Logger

  @compile {:no_warn_undefined, VintageNet}

  @ifname "wlan0"
  @cookie :wobbles_jr_cookie

  def start_link(opts), do: GenServer.start_link(__MODULE__, opts, name: __MODULE__)

  @impl true
  def init(_opts) do
    System.cmd("epmd", ["-daemon"])
    VintageNet.subscribe(addresses())
    {:ok, apply_address(%{ip: nil}, ipv4(VintageNet.get(addresses())))}
  end

  @impl true
  def handle_info({VintageNet, _name, _old, new, _meta}, state) do
    {:noreply, apply_address(state, ipv4(new))}
  end

  def handle_info(_message, state), do: {:noreply, state}

  defp addresses, do: ["interface", @ifname, "addresses"]

  defp ipv4(addresses) when is_list(addresses) do
    Enum.find_value(addresses, fn
      %{family: :inet, address: address} -> address |> :inet.ntoa() |> to_string()
      _other -> nil
    end)
  end

  defp ipv4(_addresses), do: nil

  defp apply_address(%{ip: ip} = state, ip), do: state
  defp apply_address(state, nil), do: state

  defp apply_address(state, ip) do
    if Node.alive?(), do: Node.stop()
    node = :"wobbles_jr@#{ip}"

    case Node.start(node, :longnames) do
      {:ok, _pid} ->
        Node.set_cookie(@cookie)
        Logger.info("Erlang distribution up as #{node}")

      {:error, reason} ->
        Logger.warning("Erlang distribution not started as #{node}: #{inspect(reason)}")
    end

    %{state | ip: ip}
  end
end
