defmodule Goatbot.Message.Differential do
  @moduledoc """
  How hard to turn the robot, as a difference in wheel speed.

  Published by `Goatbot.YawController` on `[:yaw]` and mixed in by
  `Goatbot.Balancer`. Positive turns the robot to its left; see the balancer
  for which wheel that speeds up on this robot.
  """

  defstruct [:differential]

  use BB.Message,
    schema: [
      differential: [
        type: :float,
        required: true,
        doc: "Wheel rad/s of differential, positive turning the robot left"
      ]
    ]

  @type t :: %__MODULE__{differential: float}
end
