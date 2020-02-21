/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <units/units.h>
#include <wpi/math>

namespace DriveTrainConstants {
    #include "Gains.h"

    constexpr int  kFrontRightMotor = 2;
    constexpr int  kFrontLeftMotor  = 0;

    constexpr int  kRearRightMotor  = 3;
    constexpr int  kRearLeftMotor   = 1;

    constexpr double kWheelCircumference = 47.87787;

    constexpr double kStabilizationP = 1;
    constexpr double kStabilizationI = 0.5;
    constexpr double kStabilizationD = 0;
    ///////////////////////

    constexpr auto kTrackwidth = 0.7_m;
    extern const frc::DifferentialDriveKinematics kDriveKinematics;

    constexpr int kEncoderCPR = 1024; //4096; //TODO: check this value
    constexpr double kWheelDiameterInches = 4;
    constexpr double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * wpi::math::pi) / static_cast<double>(kEncoderCPR);

    constexpr bool kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive. The Robot Characterization
    // Toolsuite provides a convenient tool for obtaining these values for your
    // robot.
    // constexpr auto ks = 0.664_V;
    // constexpr auto kv = 2.43 * 1_V * 1_s / 1_m;
    // constexpr auto ka = 0.407 * 1_V * 1_s * 1_s / 1_m;

    constexpr auto ks = 2.31_V;
    constexpr auto kv = 1.51 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.098 * 1_V * 1_s * 1_s / 1_m;

    // Example value only - as above, this must be tuned for your drive!
    constexpr double kPDriveVel = 4.08;
    ///////////////////////


    constexpr double kTurnP = 0.01;
    constexpr double kTurnI = 0;
    constexpr double kTurnD = 0.005;

    constexpr auto kTurnTolerance = 5_deg;
    constexpr auto kTurnRateTolerance = 10_deg_per_s;

    constexpr auto kMaxTurnRate = 100_deg_per_s;
    constexpr auto kMaxTurnAcceleration = 300_deg_per_s / 1_s;

    // MotionMagic constants
    constexpr int kSensorUnitsPerRotation = 4096;
    constexpr double kRotationsToTravel = 4;

    constexpr int kTimeoutMs = 10;
    constexpr double kNeutralDeadband = 0.001;

    static Gains kGains_Distanc = Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
    static Gains kGains_Turning = Gains( 2.0, 0.0,  4.0, 0.0,            200,  1.00 );
    static Gains kGains_Velocit = Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );
    static Gains kGains_MotProf = Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 );

    /** ---- Flat constants, you should not need to change these ---- */
    /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
    constexpr int kREMOTE_0 = 0;
    constexpr int kREMOTE_1 = 1;
    /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
    constexpr int kPID_PRIMARY = 0;
    constexpr int kPID_TURN = 1;
    /* Firmware currently supports slots [0, 3] and can be used for either PID Set */
    constexpr int kSLOT_0 = 0;
    constexpr int kSLOT_1 = 1;
    constexpr int kSLOT_2 = 2;
    constexpr int kSLOT_3 = 3;
    /* ---- Named slots, used to clarify code ---- */
    constexpr int kSlot_Distanc = kSLOT_0;
    constexpr int kSlot_Turning = kSLOT_1;
    constexpr int kSlot_Velocit = kSLOT_2;
    constexpr int kSlot_MotProf = kSLOT_3;

    constexpr int kMotionMagicSmoothing = 5;
    
}  // namespace DriveConstants

namespace AutoConstants {
    constexpr auto kMaxSpeed = 2_mps;
    constexpr auto kMaxAcceleration = 2_mps_sq;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    constexpr double kRamseteB = 2;
    constexpr double kRamseteZeta = 0.7;
}

namespace OIConstants {
    constexpr int kDriverControllerPort = 0;
}