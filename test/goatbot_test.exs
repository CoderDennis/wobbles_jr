defmodule GoatbotTest do
  use ExUnit.Case
  doctest Goatbot

  test "greets the world" do
    assert Goatbot.hello() == :world
  end
end
