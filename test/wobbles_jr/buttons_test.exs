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

  describe "double_press?/3" do
    test "the first press is never a double" do
      refute Buttons.double_press?(nil, 1_000, 400)
    end

    test "a second press inside the window is" do
      assert Buttons.double_press?(1_000, 1_400, 400)
    end

    test "a second press after it is not" do
      refute Buttons.double_press?(1_000, 1_401, 400)
    end
  end
end
