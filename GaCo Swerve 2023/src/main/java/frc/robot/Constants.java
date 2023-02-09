package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.16;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;

        // Max Physical speed/accel
        public static final double kPhysicalMaxSpeedMetersPerSecond = 3.7;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * 2 * Math.PI;
    }

    public static final class DriveConstants {

        public static final int    kNumModules = 4;
        public static final double kTrackWidth = Units.inchesToMeters(21);  // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5); // Distance between front and back wheels

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kBackRightDriveMotorPort = 7;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kBackRightTurningMotorPort = 8;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.780 + 3.1415;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 4.0725;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.8043 + 3.1415;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.5254;


        // Max Tele Drive Speeds
        public static final double kTeleMaxSpeedMetersPerSecond = ModuleConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleMaxAngularSpeedRadiansPerSecond = ModuleConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;

        // Max Tele Drive Accels
        public static final double kTeleMaxAccelerationMetersPerSecondSquared = kTeleMaxSpeedMetersPerSecond * 4;
        public static final double kTeleMaxAngularAccelerationRadiansPerSecondSquared = kTeleMaxAngularSpeedRadiansPerSecond * 4 ;
   
    }

    public static final class AutoConstants {
        // Max Tele Drive Speeds
        public static final double kAutoMaxSpeedMetersPerSecond = ModuleConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kAutoMaxAngularSpeedRadiansPerSecond = ModuleConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        public static final double kAutoMaxAccelerationMetersPerSecondSquared = kAutoMaxSpeedMetersPerSecond * 4;
        public static final double kAutoMaxAngularAccelerationRadiansPerSecondSquared = kAutoMaxAngularSpeedRadiansPerSecond * 4 ;

        public static final double kPXController = 3;
        public static final double kPYController = 2.0;
        public static final double kPHeadingController = 5;
        public static final double kTargetStandoff = 0.5;
        public static final double kTargetStandoffDeadband = 0.02;

    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
        public static final int kDriverGoToTargetButtonIdx = 5;  // PSM
        public static final int kDriverResetRobotHeadingButtonIdx = 12;  // PSM

        public static final double kDeadband = 0.05;
    }
}
