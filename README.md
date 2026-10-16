# hardware_interface_wrapper

Position and velocity state interfaces pass through the configured transmission.
The wrapped driver must expose velocity in actuator position units per second:
for example, encoder counts/s when actuator position is encoder counts. Normalize
vendor-specific fixed-point, time, or revolution units in the drive mapping first.
Joint reduction, encoder resolution, direction, and coupled-axis transformations
remain in the transmission; position offsets never apply to velocity.

Command-interface selection is unchanged. Velocity commands still pass through
without transmission conversion; this change only corrects velocity feedback.

The `test_velocity_state_transmission` GTest exercises the wrapper and loaded
transmission plugins with a test backend, without EtherCAT or robot hardware.
It covers signed reductions, position offsets, zero/nonzero velocity, coupled Z
motion, and unchanged command behavior. Run it in the owning ROS container and
retain its output and GTest XML under that repository's `output/tests/` directory.