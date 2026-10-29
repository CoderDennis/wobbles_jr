defmodule WobblesJr.Command.Explore do
  @moduledoc """
  Switches `WobblesJr.Explore` on or off, as a double press of the left button
  does.

  A command so that anything listing the robot's commands has it too. Only
  while balancing: exploring switches itself off whenever the robot stops
  balancing, so there is nothing to toggle otherwise.
  """

  use BB.Command

  @impl BB.Command
  def handle_command(_goal, context, state) do
    {:stop, :normal, %{state | result: WobblesJr.Explore.toggle(context.robot_module)}}
  end

  @impl BB.Command
  def result(%{result: result}), do: result
end
