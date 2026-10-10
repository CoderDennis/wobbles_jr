defmodule WobblesJr.Message.LeanOffset do
  @moduledoc """
  How far to move the balance loop's lean target from its setpoint.

  Published by `WobblesJr.VelocityController` on `[:lean_offset]` and added to the
  setpoint by `WobblesJr.Balancer`. Positive leans forward.
  """

  defstruct [:offset]

  use BB.Message,
    schema: [
      offset: [type: :float, required: true, doc: "Radians added to the lean setpoint"]
    ]

  @type t :: %__MODULE__{offset: float}
end
