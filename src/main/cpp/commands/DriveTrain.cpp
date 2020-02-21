#include "subsystems/DriveTrain.h"
#include <frc/SPI.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <wpi/math>
#include <frc/MotorSafety.h>

using namespace DriveTrainConstants;

DriveTrain::DriveTrain()
    : SubsystemBase(),  
      mFrontRightController{kFrontRightMotor},
      mRearRightController{kRearRightMotor},
      mFrontLeftController{kFrontLeftMotor},
      mRearLeftController{kRearLeftMotor},
      mGyro{new AHRS{frc::SPI::Port::kMXP}},
      m_odometry{frc::Rotation2d(GetHeading())} {
        // m_LeftMotor1.SetSafetyEnabled(false);
        // m_LeftMotor2.SetSafetyEnabled(false);
        // m_LeftMotor3.SetSafetyEnabled(false);
        // m_LeftMotor4.SetSafetyEnabled(false);
        // m_RightMotor1.SetSafetyEnabled(false);
        // m_RightMotor2.SetSafetyEnabled(false);
        // m_RightMotor3.SetSafetyEnabled(false);
        // m_RightMotor4.SetSafetyEnabled(false);
    // try {
    //   mGyro = new AHRS{frc::SPI::Port::kMXP};
    // } catch (std::exception ex ) {
    //           std::string err_string = "Error instantiating navX-MXP:  ";
    //           err_string += ex.what();
    //           //TODO: display error and blink LEDs RED
    //     // frc::DriverStation::ReportError(err_string.c_str());
    // }

    mFrontRightController.ConfigFactoryDefault();
    mRearRightController.ConfigFactoryDefault();

    mFrontLeftController.ConfigFactoryDefault();
    mRearLeftController.ConfigFactoryDefault();

    mFrontRightController.SetNeutralMode(NeutralMode::Brake);
    mRearRightController.SetNeutralMode(NeutralMode::Brake);

    mFrontLeftController.SetNeutralMode(NeutralMode::Brake);
    mRearLeftController.SetNeutralMode(NeutralMode::Brake);

    mFrontRightController.SetInverted(true);
    mRearRightController.SetInverted(true);

    mFrontLeftController.SetInverted(false);
    mRearLeftController.SetInverted(false);

    mFrontRightController.SetSensorPhase(false);
    mFrontLeftController.SetSensorPhase(true);

    mRearLeftController.Follow(mFrontLeftController);
	  mRearRightController.Follow(mFrontRightController);

    m_drive.SetDeadband(0);

    mFrontLeftController.ConfigSelectedFeedbackSensor(
        FeedbackDevice::QuadEncoder,
        0, 10
    );

    mRearRightController.ConfigSelectedFeedbackSensor(
        FeedbackDevice::QuadEncoder,
        0, 10
    );

    m_drive.SetSafetyEnabled(false);

    // InitMotionMagic();
    ResetEncoders();
    ResetGyro();
}

void DriveTrain::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_left_sensor_position = mFrontLeftController.GetSelectedSensorPosition(0);
  m_right_sensor_position = mFrontRightController.GetSelectedSensorPosition(0);
  frc::SmartDashboard::PutNumber("DriveTrain Left SensorPos", m_left_sensor_position/4096.0*0.15897);
  frc::SmartDashboard::PutNumber("DriveTrain Right SensorPos", m_right_sensor_position/4096.0*0.15897);
  frc::SmartDashboard::PutNumber("DriveTrain Heading", std::remainder(mGyro->GetAngle(), 360) * (kGyroReversed ? -1.0 : 1.0));
  frc::SmartDashboard::PutNumber("DriveTrain Left_Speed", (mFrontLeftController.GetSelectedSensorVelocity(0) * (1 / 4096.0) * 0.15897 * wpi::math::pi * 10));
  frc::SmartDashboard::PutNumber("DriveTrain Right_Speed", (mFrontRightController.GetSelectedSensorVelocity(0) * (1 / 4096.0) * 0.15897 * wpi::math::pi * 10));
  
  m_odometry.Update(frc::Rotation2d(GetHeading()),
                    units::meter_t(m_left_sensor_position/4096.0*0.15897),
                    units::meter_t(m_right_sensor_position/4096.0*0.15897));
 
 FeedTalons();
}

void DriveTrain::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void DriveTrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
  mFrontLeftController.SetVoltage(left);
  mFrontRightController.SetVoltage(right);
}

double DriveTrain::GetAverageEncoderDistance() {
  return (mFrontLeftController.GetSelectedSensorPosition(0)/4096.0*0.15897 + mFrontRightController.GetSelectedSensorPosition(0)/4096.0*0.15897) / 2.0;
}

void DriveTrain::FeedTalons() {
  m_drive.Feed();
}

void DriveTrain::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

void DriveTrain::ResetEncoders() {
  mFrontRightController.SetSelectedSensorPosition(0);
  mFrontLeftController.SetSelectedSensorPosition(0);
  // ZeroQuadratureEncoders();
}

units::degree_t DriveTrain::GetHeading() {
  return units::degree_t(std::remainder(mGyro->GetAngle(), 360) *
                         (kGyroReversed ? -1.0 : 1.0));
}


frc::Pose2d DriveTrain::GetPose() { return m_odometry.GetPose(); }

frc::DifferentialDriveWheelSpeeds DriveTrain::GetWheelSpeeds() {
  double encoderConstant = (1 / 4096.0) * 0.15897 * wpi::math::pi;
  return {units::meters_per_second_t(mFrontLeftController.GetSelectedSensorVelocity(0) * encoderConstant * 10),
          units::meters_per_second_t(mFrontRightController.GetSelectedSensorVelocity(0) * encoderConstant * 10)};
}

void DriveTrain::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose,
                           frc::Rotation2d(units::degree_t(GetHeading())));
}

double DriveTrain::GetTurnRate() {
  return mGyro->GetRate() * (kGyroReversed ? -1.0 : 1.0);
}

double DriveTrain::GetGyroAngle() {
  return mGyro->GetAngle();
}

void DriveTrain::ResetGyro() {
  mGyro->Reset();
}

void DriveTrain::InitMotionMagic() {
  /* Disable all motor controllers */
  mFrontLeftController.Set(ControlMode::PercentOutput, 0);
  mFrontRightController.Set(ControlMode::PercentOutput, 0);

  /* Configure the left Talon's selected sensor as local QuadEncoder */
  mFrontLeftController.ConfigSelectedFeedbackSensor(	FeedbackDevice::QuadEncoder,				// Local Feedback Source
                        kPID_PRIMARY,					// PID Slot for Source [0, 1]
                        kTimeoutMs);					// Configuration Timeout

  /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
  mFrontRightController.ConfigRemoteFeedbackFilter(mFrontLeftController.GetDeviceID(),					// Device ID of Source
                      RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor,	// Remote Feedback Source
                      kREMOTE_0,							// Source number [0, 1]
                      kTimeoutMs);						// Configuration Timeout

  /* Setup Sum signal to be used for Distance */
		mFrontRightController.ConfigSensorTerm(SensorTerm::SensorTerm_Sum0, FeedbackDevice::RemoteSensor0, kTimeoutMs);				// Feedback Device of Remote Talon
		mFrontRightController.ConfigSensorTerm(SensorTerm::SensorTerm_Sum1, FeedbackDevice::CTRE_MagEncoder_Relative, kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Setup Difference signal to be used for Turn */
		mFrontRightController.ConfigSensorTerm(SensorTerm::SensorTerm_Diff1, FeedbackDevice::RemoteSensor0, kTimeoutMs);
		mFrontRightController.ConfigSensorTerm(SensorTerm::SensorTerm_Diff0, FeedbackDevice::CTRE_MagEncoder_Relative, kTimeoutMs);
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		mFrontRightController.ConfigSelectedFeedbackSensor(	FeedbackDevice::SensorSum, 
													kPID_PRIMARY,
													kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		mFrontRightController.ConfigSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														kPID_PRIMARY,		// PID Slot of Source 
														kTimeoutMs);		// Configuration Timeout
		
		/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
		mFrontRightController.ConfigSelectedFeedbackSensor(	FeedbackDevice::SensorDifference, 
													kPID_TURN, 
													kTimeoutMs);
		
		/* Scale the Feedback Sensor using a coefficient */
		mFrontRightController.ConfigSelectedFeedbackCoefficient(	1,
														kPID_TURN, 
														kTimeoutMs);
		
		/* Set status frame periods to ensure we don't have stale data */
		mFrontRightController.SetStatusFramePeriod(StatusFrame::Status_12_Feedback1_, 20, kTimeoutMs);
		mFrontRightController.SetStatusFramePeriod(StatusFrame::Status_13_Base_PIDF0_, 20, kTimeoutMs);
		mFrontRightController.SetStatusFramePeriod(StatusFrame::Status_14_Turn_PIDF1_, 20, kTimeoutMs);
		mFrontRightController.SetStatusFramePeriod(StatusFrame::Status_10_Targets_, 20, kTimeoutMs);
		mFrontLeftController.SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 5, kTimeoutMs);

		/* Configure neutral deadband */
		mFrontRightController.ConfigNeutralDeadband(kNeutralDeadband, kTimeoutMs);
		mFrontLeftController.ConfigNeutralDeadband(kNeutralDeadband, kTimeoutMs);
		
		/* Motion Magic Configurations */
		mFrontRightController.ConfigMotionAcceleration(2000, kTimeoutMs);
		mFrontRightController.ConfigMotionCruiseVelocity(2000, kTimeoutMs);

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		mFrontLeftController.ConfigPeakOutputForward(+1.0, kTimeoutMs);
		mFrontLeftController.ConfigPeakOutputReverse(-1.0, kTimeoutMs);
		mFrontRightController.ConfigPeakOutputForward(+1.0, kTimeoutMs);
		mFrontRightController.ConfigPeakOutputReverse(-1.0, kTimeoutMs);

		/* FPID Gains for distance servo */
		mFrontRightController.Config_kP(kSlot_Distanc, kGains_Distanc.kP, kTimeoutMs);
		mFrontRightController.Config_kI(kSlot_Distanc, kGains_Distanc.kI, kTimeoutMs);
		mFrontRightController.Config_kD(kSlot_Distanc, kGains_Distanc.kD, kTimeoutMs);
		mFrontRightController.Config_kF(kSlot_Distanc, kGains_Distanc.kF, kTimeoutMs);
		mFrontRightController.Config_IntegralZone(kSlot_Distanc, kGains_Distanc.kIzone, kTimeoutMs);
		mFrontRightController.ConfigClosedLoopPeakOutput(kSlot_Distanc, kGains_Distanc.kPeakOutput, kTimeoutMs);
		mFrontRightController.ConfigAllowableClosedloopError(kSlot_Distanc, 0, kTimeoutMs);

		/* FPID Gains for turn servo */
		mFrontRightController.Config_kP(kSlot_Turning, kGains_Turning.kP, kTimeoutMs);
		mFrontRightController.Config_kI(kSlot_Turning, kGains_Turning.kI, kTimeoutMs);
		mFrontRightController.Config_kD(kSlot_Turning, kGains_Turning.kD, kTimeoutMs);
		mFrontRightController.Config_kF(kSlot_Turning, kGains_Turning.kF, kTimeoutMs);
		mFrontRightController.Config_IntegralZone(kSlot_Turning, (int)kGains_Turning.kIzone, kTimeoutMs);
		mFrontRightController.ConfigClosedLoopPeakOutput(kSlot_Turning, kGains_Turning.kPeakOutput, kTimeoutMs);
		mFrontRightController.ConfigAllowableClosedloopError(kSlot_Turning, 0, kTimeoutMs);

		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		mFrontRightController.ConfigClosedLoopPeriod(0, closedLoopTimeMs, kTimeoutMs);
		mFrontRightController.ConfigClosedLoopPeriod(1, closedLoopTimeMs, kTimeoutMs);

		/**
		 * configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		mFrontRightController.ConfigAuxPIDPolarity(false, kTimeoutMs);

		/* Initialize */
		// _firstCall = true;
		// _state = false;
		mFrontRightController.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 10);

    mFrontRightController.ConfigMotionSCurveStrength(kMotionMagicSmoothing);

		ZeroQuadratureEncoders();
}

void DriveTrain::ZeroQuadratureEncoders() {
    mFrontRightController.GetSensorCollection().SetQuadraturePosition(0, kTimeoutMs);
    mFrontLeftController.GetSensorCollection().SetQuadraturePosition(0, kTimeoutMs);
}

void DriveTrain::InitMotionMagicProfileSlot() {
    /* Determine which slot affects which PID */
    mFrontRightController.SelectProfileSlot(kSlot_Distanc, kPID_PRIMARY);
    mFrontRightController.SelectProfileSlot(kSlot_Turning, kPID_TURN);
    m_TargetAngle = mFrontRightController.GetSelectedSensorPosition(1);
}

void DriveTrain::MoveWithMotionMagic(double target) {
    /* Calculate targets from gamepad inputs */
    double target_sensorUnits = kSensorUnitsPerRotation * (target / kWheelCircumference);
    double target_turn = m_TargetAngle;
    
    /* Configured for MotionMagic on Quad Encoders' Sum and Auxiliary PID on Quad Encoders' Difference */
    mFrontRightController.Set(ControlMode::MotionMagic, target_sensorUnits, DemandType::DemandType_AuxPID, target_turn);
    mFrontLeftController.Follow(mFrontRightController, FollowerType::FollowerType_AuxOutput1);
}

