// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelRadiusMeters = 0.0508;
        public static final int kEncoderResolution = 2048;
        public static final double kDriveMotorGearRatio = (48./16.)*(16./28.)*(60./15.);
        public static final double kTurningMotorGearRatio = (32./15.)*(60./15.);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelRadiusMeters * 2;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {
        public static final Translation2d m_frontLeftLocation = new Translation2d(-0.381, 0.381);
        public static final Translation2d m_frontRightLocation = new Translation2d(-0.381, -0.381);
        public static final Translation2d m_backLeftLocation = new Translation2d(0.381, 0.381);
        public static final Translation2d m_backRightLocation = new Translation2d(0.381, -0.381);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 7;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kBackLeftTurningMotorPort = 4;
        public static final int kFrontRightTurningMotorPort = 6;
        public static final int kBackRightTurningMotorPort = 8;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.678;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.110;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.389;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.303;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }

}
