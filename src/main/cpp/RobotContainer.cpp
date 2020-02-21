/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"
#include <frc2/command/button/JoystickButton.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RamseteCommand.h>

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

   m_Drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_Drive.ArcadeDrive(
          m_driverController.GetX(frc::GenericHID::kLeftHand),
          -1.0 * m_driverController.GetY(frc::GenericHID::kLeftHand));
    },
    {&m_Drive}));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(
          DriveTrainConstants::ks, DriveTrainConstants::kv, DriveTrainConstants::ka),
      DriveTrainConstants::kDriveKinematics, 10_V);

  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(DriveTrainConstants::kDriveKinematics);
  // Apply the voltage constraint
  config.AddConstraint(autoVoltageConstraint);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(0.5_m, 0.5_m), frc::Translation2d(1_m, -0.5_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(1.5_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  frc2::RamseteCommand ramseteCommand(
      exampleTrajectory, [this]() { return m_Drive.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveTrainConstants::ks, DriveTrainConstants::kv, DriveTrainConstants::ka),
      DriveTrainConstants::kDriveKinematics,
      [this] { return m_Drive.GetWheelSpeeds(); },
      frc2::PIDController(DriveTrainConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveTrainConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_Drive.TankDriveVolts(left, right); },
      {&m_Drive});

  std::cout << "finished inst ramseteCommand" << std::endl;

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(ramseteCommand),
      frc2::InstantCommand([this] { m_Drive.TankDriveVolts(0_V, 0_V); }, {}));
}
