#pragma once

#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/SubsystemBase.h>
#include <units/units.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>

#include <ctre/Phoenix.h>
#include "AHRS.h"

#include "Constants.h"

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  void Periodic() override;

  void ArcadeDrive(double fwd, double rot);
  void ResetEncoders();
  void ResetGyro();
	double GetGyroAngle();
  units::degree_t GetHeading();
  double GetTurnRate();
  double GetDistance();

  double GetAverageEncoderDistance();
  void SetMaxOutput(double maxOutput);

  void TankDriveVolts(units::volt_t left, units::volt_t right);
  frc::Pose2d GetPose();
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();
  void ResetOdometry(frc::Pose2d pose);

  void InitMotionMagic();
  void ZeroQuadratureEncoders();
  void InitMotionMagicProfileSlot();
  void MoveWithMotionMagic(double target);

  //void Stop();

  void FeedTalons();

 private:
    WPI_TalonSRX mFrontRightController;
    WPI_TalonSRX mRearRightController;

    WPI_TalonSRX mFrontLeftController;
    WPI_TalonSRX mRearLeftController;

    double m_left_sensor_position;
    double m_right_sensor_position;

    frc::SpeedControllerGroup m_RightSide{mFrontRightController, mRearRightController};
    frc::SpeedControllerGroup m_LeftSide{mFrontLeftController, mRearLeftController};
    
    frc::DifferentialDrive m_drive{m_LeftSide, m_RightSide};

    AHRS *mGyro = nullptr;

    frc::DifferentialDriveOdometry m_odometry;

    double m_TargetAngle = 0.0;
};