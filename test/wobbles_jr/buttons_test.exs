defmodule WobblesJr.ButtonsTest do
  use ExUnit.Case, async: true

  alias WobblesJr.Buttons

  describe "on_press/1" do
    test "the left button stands the robot at once" do
      assert Buttons.on_press(3) == {:now, :stand}
    end

    test "the middle button disarms at once" do
      assert Buttons.on_press(2) == {:now, :disarm}
    end

    test "the right button powers off only once held" do
      assert Buttons.on_press(1) == {:hold, :poweroff}
    end
  end
end
