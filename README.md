# WobblesJr

This is my code for the balance bot we built at the workshop led by James Harton at Goatmire 2026.

I also copied over the balance controllers and other changes from https://github.com/gworkman/goatbot

Changes I made include the following:
- Adds a parameter for LED brightness because I found the default to be too bright.
- Shows battery charge status on the LEDs while disarmed and on USB.
- Gives the board's three buttons actions (see below).

From https://github.com/lawik/goatbot:
- A face on the e-ink panel while balancing or fallen. Disarmed or idle, it keeps the status screen.
- Explore mode, started with a double press of the left button.

## Buttons

Seen from the front, left to right:

| Button | Action |
|---|---|
| Left | **Stand.** Arms the robot first if it isn't armed. Hold it upright before pressing. |
| Left, double press | **Explore on or off.** Wanders like an old robot vacuum: forward until it bumps into something, then backs off and turns. From idle, the first press stands it up and it starts exploring once it's balancing. A fall or disarm stops it. |
| Middle | **Disarm.** The motors cut out, so catch the robot. |
| Right | **Power off**, after holding it for 2 seconds. Only works while disarmed. |

The STM32 numbers the buttons right to left (1 is the right-hand button), so
in the code the left button is 3. See `lib/wobbles_jr/buttons.ex`.

## Targets

Nerves applications produce images for hardware targets based on the
`MIX_TARGET` environment variable. If `MIX_TARGET` is unset, `mix` builds an
image that runs on the host (e.g., your laptop). This is useful for executing
logic tests, running utilities, and debugging. Other targets are represented by
a short name like `rpi5` that maps to a Nerves system image for that platform.
All of this logic is in the generated `mix.exs` and may be customized. For more
information about targets see:

https://nerves.hexdocs.pm/supported-targets.html

## Getting Started

To start your Nerves app:
  * `export MIX_TARGET=my_target` or prefix every command with
    `MIX_TARGET=my_target`. For example, `MIX_TARGET=rpi5`
  * Install dependencies with `mix deps.get`
  * Create firmware with `mix firmware`
  * Burn to an SD card with `mix burn`

## Learn more

  * Official docs: https://nerves.hexdocs.pm/getting-started.html
  * Official website: https://nerves-project.org/
  * Forum: https://elixirforum.com/c/nerves-forum
  * Elixir Discord #nerves channel: https://discord.gg/elixir
  * Source: https://github.com/nerves-project/nerves

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE).
