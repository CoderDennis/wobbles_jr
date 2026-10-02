defmodule WobblesJrTest do
  use ExUnit.Case
  doctest WobblesJr

  test "greets the world" do
    assert WobblesJr.hello() == :world
  end
end
