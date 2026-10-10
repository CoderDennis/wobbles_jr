defmodule WobblesJr.Message.Button do
  @moduledoc """
  A press or release of one of the board's three buttons.

  Published by `WobblesJr.Sensor.Power` on `[:buttons]`, from the STM32's
  `EVT BTN` events. Buttons are numbered 1 to 3, matching the demo's
  `:btn1`..`:btn3` (the STM32 reports them as pins 0 to 2).
  """

  defstruct [:button, :pressed]

  use BB.Message,
    schema: [
      button: [type: {:in, [1, 2, 3]}, required: true, doc: "Which button, 1 to 3"],
      pressed: [type: :boolean, required: true, doc: "True when pressed, false when released"]
    ]

  @type t :: %__MODULE__{button: 1..3, pressed: boolean}
end
