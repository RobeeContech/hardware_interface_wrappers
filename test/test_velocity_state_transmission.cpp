// Copyright 2026 RobeeRobotics LTD
// Licensed under the Apache License, Version 2.0.

#include <array>
#include <cmath>
#include <string>

#include <gtest/gtest.h>
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface_wrappers/transmission_wrapper.hpp"

namespace
{

// Both commands are required by the existing loopback test backend. Velocity
// commands remain raw, allowing the test to inject actuator velocity feedback.
hardware_interface::HardwareInfo make_info(const std::string & transmissions)
{
    const std::string urdf = R"(
      <robot name="velocity_test">
        <ros2_control name="velocity_test" type="system">
          <hardware>
            <plugin>hardware_interface_wrappers/TransmissionWrapper</plugin>
            <param name="wrapped_interface">test_hardware_interface_wrappers/TestableJointSystem</param>
          </hardware>
          <joint name="joint1">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
          </joint>
          <joint name="joint2">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
          </joint>
    )" + transmissions + "</ros2_control></robot>";
    return hardware_interface::parse_control_resources_from_urdf(urdf).at(0);
}

}  // namespace

TEST(VelocityStateTransmission, SignedReductionWithoutPositionOffset)
{
    const auto info = make_info(R"(
      <transmission name="positive">
        <plugin>robee_transmission_interface/BasicTransmission</plugin>
        <joint name="joint1" role="joint1">
          <mechanical_reduction>2</mechanical_reduction>
          <offset>0.5</offset>
        </joint>
      </transmission>
      <transmission name="negative">
        <plugin>robee_transmission_interface/BasicTransmission</plugin>
        <joint name="joint2" role="joint2">
          <mechanical_reduction>-4</mechanical_reduction>
          <offset>-0.25</offset>
        </joint>
      </transmission>
    )");
    hardware_interface_wrappers::TransmissionWrapper wrapper;
    ASSERT_EQ(wrapper.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
    auto states = wrapper.export_state_interfaces();
    auto commands = wrapper.export_command_interfaces();
    ASSERT_EQ(states.size(), 4u);
    ASSERT_EQ(commands.size(), 4u);
    ASSERT_EQ(states[1].get_name(), "joint1/velocity");
    ASSERT_EQ(states[3].get_name(), "joint2/velocity");
    commands[0].set_value(1.25);
    commands[2].set_value(-0.75);

    for (double raw_velocity : std::array<double, 3>{8.0, -8.0, 0.0}) {
        SCOPED_TRACE(raw_velocity);
        commands[1].set_value(raw_velocity);
        commands[3].set_value(raw_velocity);
        ASSERT_EQ(wrapper.write(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
                  hardware_interface::return_type::OK);
        ASSERT_EQ(wrapper.read(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
                  hardware_interface::return_type::OK);
        EXPECT_DOUBLE_EQ(states[0].get_value(), 1.25);
        EXPECT_DOUBLE_EQ(states[2].get_value(), -0.75);
        EXPECT_DOUBLE_EQ(states[1].get_value(), raw_velocity / 2.0);
        EXPECT_DOUBLE_EQ(states[3].get_value(), raw_velocity / -4.0);
        EXPECT_DOUBLE_EQ(commands[1].get_value(), raw_velocity);
        EXPECT_DOUBLE_EQ(commands[3].get_value(), raw_velocity);
    }
}

TEST(VelocityStateTransmission, CoupledZVelocityUsesBothActuators)
{
    const auto info = make_info(R"(
      <transmission name="coupled_z">
        <plugin>robee_transmission_interface/ScaraTransmission</plugin>
        <actuator name="motor1" role="motor1"/>
        <actuator name="motor2" role="motor2"/>
        <joint name="joint1" role="joint1">
          <mechanical_reduction>2</mechanical_reduction>
          <offset>0</offset>
        </joint>
        <joint name="joint2" role="joint2">
          <mechanical_reduction>4</mechanical_reduction>
          <offset>0</offset>
        </joint>
        <param name="ppr1">4000</param>
        <param name="ppr2">8000</param>
        <param name="screw_reduction">250</param>
      </transmission>
    )");
    hardware_interface_wrappers::TransmissionWrapper wrapper;
    ASSERT_EQ(wrapper.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
    auto states = wrapper.export_state_interfaces();
    auto commands = wrapper.export_command_interfaces();
    ASSERT_EQ(states.size(), 4u);
    ASSERT_EQ(commands.size(), 4u);
    commands[0].set_value(0.02);
    commands[2].set_value(0.5);
    const double pi = std::acos(-1.0);

    // Geared motor speeds in rad/s: differential motion, common motion,
    // reverse differential motion, then rest.
    const std::array<std::array<double, 2>, 4> speeds{{
        {2.0, 0.5}, {0.5, 0.5}, {-2.0, -0.5}, {0.0, 0.0}}};
    for (const auto & speed : speeds) {
        SCOPED_TRACE(speed[0]);
        commands[1].set_value(speed[0] * 2.0 * 4000.0 / (2.0 * pi));
        commands[3].set_value(speed[1] * 4.0 * 8000.0 / (2.0 * pi));
        ASSERT_EQ(wrapper.write(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
                  hardware_interface::return_type::OK);
        ASSERT_EQ(wrapper.read(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
                  hardware_interface::return_type::OK);
        EXPECT_NEAR(states[0].get_value(), 0.02, 1e-12);
        EXPECT_NEAR(states[2].get_value(), 0.5, 1e-12);
        EXPECT_NEAR(states[1].get_value(), (speed[0] - speed[1]) / 250.0, 1e-12);
        EXPECT_NEAR(states[3].get_value(), speed[1], 1e-12);
    }
}
