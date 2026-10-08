defmodule Goatbot.Robot do
  use BB, extensions: [BB.Parameter.Store.CubDB.Dsl]

  import BB.Unit

  commands do
    command :arm do
      handler(BB.Command.Arm)
      allowed_states([:disarmed])
    end

    command :disarm do
      handler(BB.Command.Disarm)
      allowed_states([:idle])
    end

    # Disarmed only: taking the operating system out from under a balancing
    # robot drops it on the floor.
    command :poweroff do
      handler(BB.NSK.Command.Poweroff)
      allowed_states([:disarmed])
    end
  end

  topology do
    # The robot is not bolted to anything, so the chain starts at the world
    # and reaches the body through the two ways it can move: across the
    # ground, and leaning about the wheel axis. Neither joint has an
    # actuator, and with no encoders the ground pose stays at identity —
    # only the lean is observable.
    link :world do
      joint :ground do
        type(:planar)

        axis do
        end

        link :ground_contact do
          joint :lean do
            type(:revolute)
            # Pitch about +Y, so leaning forwards is positive. The axis
            # sits a wheel radius above the ground — half of a 43mm wheel,
            # and anything under 32mm leaves the body resting on the floor
            # with the wheels spinning in the air.
            axis(roll: ~u(-90 degree))
            origin(z: ~u(21.5 millimeter))

            limit(
              lower: ~u(-90 degree),
              upper: ~u(90 degree),
              effort: ~u(0 newton_meter),
              velocity: ~u(0 radian_per_second)
            )

            link :base_link do
              # From the CAD model, measured against the wheel axis.
              visual do
                box(x: ~u(20.7 millimeter), y: ~u(99 millimeter), z: ~u(124 millimeter))
                origin(x: ~u(1.75 millimeter), z: ~u(46 millimeter))
              end

              inertial do
                # The height is measured. The fore-aft is not: it is
                # back-derived from the lean the robot actually balances at,
                # so the geometry and the `:balance` setpoint tell the same
                # story. See `mix help bb_nsk.install`.
                origin(x: ~u(2.4 millimeter), z: ~u(45.5 millimeter))

                # Weighed with the wheels off: their mass sits on the axle
                # where it makes no toppling torque, so the body alone is
                # the pendulum.
                mass(~u(138 gram))

                # A uniform box of the body's dimensions about its centre of
                # mass. An approximation — the panel is on the front and the
                # battery is one lump — but not a negligible one: the body's
                # own pitch inertia is a third of the total about the wheel
                # axis.
                inertia(
                  ixx: ~u(2895 gram_square_centimeter),
                  iyy: ~u(1818 gram_square_centimeter),
                  izz: ~u(1176 gram_square_centimeter),
                  ixy: ~u(0 gram_square_centimeter),
                  ixz: ~u(0 gram_square_centimeter),
                  iyz: ~u(0 gram_square_centimeter)
                )
              end

              joint :left_wheel_joint do
                type(:continuous)
                axis(roll: ~u(-90 degree))
                origin(y: ~u(55 millimeter))

                limit(effort: ~u(0.2 newton_meter), velocity: ~u(20 radian_per_second))

                actuator(
                  :left_wheel,
                  {BB.NSK.Wheel,
                   forward_pwm: 3,
                   reverse_pwm: 2,
                   enable_pin: "PE6",
                   deadband: param([:motor, :deadband])}
                )

                link :left_wheel_link do
                  inertial do
                    mass(~u(15 gram))

                    inertia(
                      ixx: ~u(17.33 gram_square_centimeter),
                      iyy: ~u(34.67 gram_square_centimeter),
                      izz: ~u(17.33 gram_square_centimeter),
                      ixy: ~u(0 gram_square_centimeter),
                      ixz: ~u(0 gram_square_centimeter),
                      iyz: ~u(0 gram_square_centimeter)
                    )
                  end
                end
              end

              joint :right_wheel_joint do
                type(:continuous)
                axis(roll: ~u(-90 degree))
                origin(y: ~u(-55 millimeter))

                limit(effort: ~u(0.2 newton_meter), velocity: ~u(20 radian_per_second))

                actuator(
                  :right_wheel,
                  {BB.NSK.Wheel,
                   forward_pwm: 5,
                   reverse_pwm: 4,
                   enable_pin: "PE11",
                   deadband: param([:motor, :deadband])}
                )

                link :right_wheel_link do
                  inertial do
                    mass(~u(15 gram))

                    inertia(
                      ixx: ~u(17.33 gram_square_centimeter),
                      iyy: ~u(34.67 gram_square_centimeter),
                      izz: ~u(17.33 gram_square_centimeter),
                      ixy: ~u(0 gram_square_centimeter),
                      ixz: ~u(0 gram_square_centimeter),
                      iyz: ~u(0 gram_square_centimeter)
                    )
                  end
                end
              end
            end
          end
        end
      end
    end
  end

  # `/root` is the Nerves application data partition and does not exist on a
  # host. The DSL is compiled, so this is decided when the firmware is built.
  #
  # `tmp/` rather than `_build/` on the host — which is what
  # `bb_parameter_store_cubdb` would pick for a project with no Nerves in it
  # — because these are tuned values and `mix clean` should not take them.
  parameter_store_cubdb do
    data_dir(
      if Mix.target() == :host,
        do: "tmp/goatbot_params",
        else: "/root/goatbot_params"
    )
  end

  parameters do
    group :motor do
      # Measured unloaded, so the figure under the robot's own weight will be
      # higher — which is why it is tunable.
      param(:deadband,
        type: :float,
        default: 0.01,
        min: 0.0,
        max: 0.9,
        doc: "Duty below which the motor doesn't turn, as a fraction of full scale"
      )
    end
  end
end
