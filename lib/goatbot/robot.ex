defmodule Goatbot.Robot do
  use BB, extensions: [BB.Parameter.Store.CubDB.Dsl]

  import BB.Unit

  commands do
    command :arm do
      # A custom handler so that arming lands in the state the robot's
      # attitude calls for rather than always in `:idle`. `arm true` keeps
      # `BB.Safety.arm/1` routing through the command system.
      handler(BB.NSK.Command.Arm)
      allowed_states([:disarmed])
      arm(true)
    end

    command :disarm do
      handler(BB.Command.Disarm)
      allowed_states([:idle, :balancing, :fallen])
    end

    # Disarmed only: taking the operating system out from under a balancing
    # robot drops it on the floor.
    command :poweroff do
      handler(BB.NSK.Command.Poweroff)
      allowed_states([:disarmed])
    end

    command :stand do
      handler(BB.NSK.Command.Stand)
      allowed_states([:idle, :fallen])
    end

    # From `:idle` as well as `:balancing`, because a robot armed while
    # already on its side has fallen over without ever having balanced.
    command :fall do
      handler(BB.NSK.Command.Fall)
      allowed_states([:balancing, :idle])
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

              # The chip publishes in its own axes and a `sensor` has no origin — the
              # link it hangs off is its frame — so the mounting is links of its own and
              # everything downstream gets the transform from the kinematics.
              #
              # **Two joints rather than one origin carrying both**, because
              # `BB.Math.Transform.from_origin/1` composes the rotation before the
              # translation: one origin with both would put the chip 29.5mm *forward* of
              # the wheel axis instead of above it.
              joint :imu_mount_joint do
                type(:fixed)
                origin(x: ~u(-4.9 millimeter), z: ~u(29.5 millimeter))

                link :imu_mount do
                  joint :imu_joint do
                    type(:fixed)
                    origin(pitch: ~u(90 degree), yaw: ~u(-90 degree))

                    link :imu_link do
                      # The driver stops rather than declining when the chip isn't
                      # there, which on a host would take the supervision tree with it.
                      if Mix.target() != :host do
                        # No magnetometer, so the chip publishes an identity
                        # orientation and this fuses a real one.
                        sensor :imu,
                               {BB.Sensor.BMI323,
                                bus: "i2c-0",
                                address: 0x68,
                                mode: :polling,
                                accelerometer_range: 4,
                                accelerometer_odr: 200,
                                gyroscope_range: 500,
                                gyroscope_odr: 200,
                                publish_rate: param([:sampling, :publish_rate])} do
                          estimator(
                            :orientation,
                            {BB.Estimator.Ahrs.Mahony,
                             kp: param([:ahrs, :kp]), ki: param([:ahrs, :ki])}
                          )
                        end
                      end
                    end
                  end
                end
              end
            end

            # No actuator, but the lean is observable: the IMU hangs off a link below
            # this joint, and what it reports about gravity is exactly this joint's
            # configuration.
            sensor(:lean_angle, BB.NSK.Sensor.Lean)
          end
        end

        # The heading is observable even though the position isn't: the gyro
        # measures yaw rate directly, and integrating it is a real angle where dead
        # reckoning `x` and `y` from wheel commands would be a fiction. So this
        # joint's `theta` means something and its translation still doesn't.
        sensor(:heading, BB.NSK.Sensor.Heading)
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

    group :sampling do
      # This is the balance loop's rate. The robot falls with a 68ms time
      # constant, so samples per fall matter more than they sound — see
      # `mix help bb_nsk.add_imu`.
      #
      # The chip's ODR is 200 Hz, so going above that repeats samples unless the
      # ODR below is raised with it.
      param(:publish_rate,
        type: {:unit, :hertz},
        default: ~u(200 hertz),
        min: ~u(10 hertz),
        max: ~u(400 hertz),
        doc: "How often the IMU publishes, which is the rate the balance loop runs at"
      )
    end

    group :ahrs do
      # The filter sits *inside* the control loop, so its lag is part of the
      # plant the balance gains are tuned against — changing one invalidates the
      # tuning of the other.
      param(:kp,
        type: :float,
        default: 0.4,
        min: 0.0,
        max: 5.0,
        doc:
          "How hard the accelerometer may pull the estimate. Small: it can't tell lean from acceleration"
      )

      # Defaults to zero in the library, so it has to be set.
      param(:ki,
        type: :float,
        default: 0.005,
        min: 0.0,
        max: 0.5,
        doc: "Gyroscope bias estimation. Zero is proportional-only, which drifts"
      )
    end

    group :balance do
      # Not zero: the centre of mass sits ahead of the wheel axis, so the robot
      # is only in equilibrium leaning back far enough to bring it over them.
      param(:setpoint,
        type: {:unit, :degree},
        default: ~u(-3 degree),
        min: ~u(-30 degree),
        max: ~u(30 degree),
        doc: "The lean the balance controller holds, negative leaning back"
      )

      # Found on the robot rather than guessed, and paired with
      # `derivative_gain` — what matters is the ratio. The sweeps behind both are
      # in `mix help bb_nsk.add_balance`.
      param(:proportional_gain,
        type: :float,
        default: 180.0,
        min: 0.0,
        max: 250.0,
        doc: "Wheel velocity per radian of lean error, in 1/s"
      )

      # A sharp optimum, not a plateau. Above 4.0 the derivative term amplifies
      # gyro noise into the command and the wheels clip.
      param(:derivative_gain,
        type: :float,
        default: 4.0,
        min: 0.0,
        max: 20.0,
        doc: "Wheel velocity per radian per second of lean rate"
      )

      # More than a bias corrector: the commanded velocity is the only estimate
      # of speed this hardware can produce, so this is the only velocity feedback
      # in the system. Without it the robot rocks as it accelerates away.
      param(:trim_gain,
        type: :float,
        default: 0.01,
        min: 0.0,
        max: 1.0,
        doc: "How fast the setpoint chases away a persistent wheel command. Slow on purpose"
      )

      param(:trim_limit,
        type: {:unit, :degree},
        default: ~u(5 degree),
        min: ~u(0 degree),
        max: ~u(15 degree),
        doc: "How far the trim may wander before it is telling you something else is wrong"
      )

      param(:fall_angle,
        type: {:unit, :degree},
        default: ~u(30 degree),
        min: ~u(5 degree),
        max: ~u(80 degree),
        doc: "Lean error past which it gives up and waits to be stood back up"
      )

      # Tight on purpose. Any looser and the robot takes over while still moving
      # in the operator's hand, and then spends its life answering that
      # transient rather than balancing.
      param(:catch_angle,
        type: {:unit, :degree},
        default: ~u(2 degree),
        min: ~u(1 degree),
        max: ~u(30 degree),
        doc: "Lean error within which it will take over again"
      )

      param(:catch_rate,
        type: {:unit, :degree_per_second},
        default: ~u(4 degree_per_second),
        min: ~u(1 degree_per_second),
        max: ~u(200 degree_per_second),
        doc: "How still it must be held before taking over, so it can't grab mid-air"
      )
    end

    group :drive do
      # Parameter names are unique across *every* group, not just within one, so
      # these cannot be `:limit` and `:slew` — `:yaw` already has a `:limit`.
      param(:authority,
        type: {:unit, :degree},
        default: ~u(2 degree),
        min: ~u(0 degree),
        max: ~u(12 degree),
        doc: "Lean a full throttle may spend. Bounds acceleration, not speed"
      )

      param(:ramp,
        type: {:unit, :degree_per_second},
        default: ~u(8 degree_per_second),
        min: ~u(1 degree_per_second),
        max: ~u(90 degree_per_second),
        doc: "How fast the drive lean may change, so a thumb slammed over ramps rather than steps"
      )

      # Four times `ramp`: slow to wind on so a thumb slammed over does not lurch,
      # fast to unwind so letting go stops the robot promptly. If letting go
      # still feels slow, this is the knob.
      param(:release,
        type: {:unit, :degree_per_second},
        default: ~u(32 degree_per_second),
        min: ~u(1 degree_per_second),
        max: ~u(180 degree_per_second),
        doc: "How fast the drive lean returns towards zero, which is how quickly letting go stops"
      )
    end

    group :yaw do
      # Holds a heading, so a wheel finding more grip than the other doesn't
      # quietly turn the robot.
      #
      # **Both signs are confirmed on hardware**, and separately: a loop with
      # both inverted behaves exactly like one with both right, and no
      # closed-loop test can tell them apart.
      param(:gain,
        type: :float,
        default: 16.0,
        min: 0.0,
        max: 50.0,
        doc: "Differential wheel velocity per radian of heading error, in 1/s"
      )

      # Never swept — part of a tested configuration rather than a tested value.
      param(:damping,
        type: :float,
        default: 0.5,
        min: 0.0,
        max: 20.0,
        doc: "Differential wheel velocity per radian per second of yaw rate"
      )

      # Velocity spent turning is velocity unavailable for staying upright, and
      # straightening up is not worth falling over for.
      param(:limit,
        type: :float,
        default: 3.0,
        min: 0.0,
        max: 15.0,
        doc: "How much wheel velocity the heading may spend, in rad/s"
      )

      # No magnetometer, so the heading drifts and the held heading must follow
      # it. The trade: a disturbance is corrected only to the extent it happens
      # faster than this. Short is drift-tolerant and forgetful, long is patient
      # and slowly turns.
      param(:tau,
        type: :float,
        default: 10.0,
        min: 0.5,
        max: 120.0,
        doc: "Seconds for the held heading to follow the measured one"
      )
    end
  end

  controllers do
    controller(
      :balancer,
      {BB.NSK.Balance.Controller,
       setpoint: param([:balance, :setpoint]),
       proportional_gain: param([:balance, :proportional_gain]),
       derivative_gain: param([:balance, :derivative_gain]),
       trim_gain: param([:balance, :trim_gain]),
       trim_limit: param([:balance, :trim_limit]),
       fall_angle: param([:balance, :fall_angle]),
       catch_angle: param([:balance, :catch_angle]),
       catch_rate: param([:balance, :catch_rate]),
       yaw_gain: param([:yaw, :gain]),
       yaw_damping: param([:yaw, :damping]),
       yaw_limit: param([:yaw, :limit]),
       yaw_tau: param([:yaw, :tau]),
       drive_limit: param([:drive, :authority]),
       drive_slew: param([:drive, :ramp]),
       drive_release: param([:drive, :release])}
    )

    controller(:leds, {BB.NSK.Leds.Controller, brightness: 40})
  end

  states do
    state(:balancing, doc: "Actively holding itself upright")
    state(:fallen, doc: "Past recovering, wheels braked, waiting to be stood back up")
  end
end
