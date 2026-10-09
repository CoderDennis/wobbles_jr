defmodule Goatbot.RemoteTest do
  use ExUnit.Case, async: true

  alias Goatbot.Remote

  describe "command/3" do
    test "passes the throttle through and scales the turn by the turn rate" do
      assert Remote.command(0.5, -0.25, 2.0) == {0.5, -0.5}
    end

    test "clamps both to -1..1 before scaling" do
      assert Remote.command(3, -7, 2.0) == {1.0, -2.0}
    end

    test "answers floats for integer input" do
      assert Remote.command(0, 1, 2.0) == {0.0, 2.0}
    end

    test "reads anything that is not a number as zero" do
      assert Remote.command(:fast, "left", 2.0) == {0.0, 0.0}
    end
  end

  describe "describe/1" do
    test "a finished command is ok" do
      assert Remote.describe({:ok, :balancing}) == "ok"
      assert Remote.describe({:ok, :idle, next_state: :idle}) == "ok"
    end

    test "a failure is its reason, clipped for the badge" do
      assert Remote.describe({:error, :not_armed}) == ":not_armed"
      assert Remote.describe({:error, %RuntimeError{message: "nope"}}) == "nope"
      assert byte_size(Remote.describe({:error, String.duplicate("x", 100)})) == 24
    end
  end
end
