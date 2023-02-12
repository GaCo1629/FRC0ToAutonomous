package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class VisionConstants {
        public static String cameraName = "OV5647";
        static final Transform3d robotToCam =
        new Transform3d(
                new Translation3d(0.3, 0.0, 0.3),
                new Rotation3d(
                        0, 0,
                        0)); // Cam mounted facing forward, 30cm forward of center, 30cm up from center.

    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.31;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;

        // Max Physical speed/accel
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6.0;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * 2 * Math.PI;
    }

    public static final class DriveConstants {

        public static final int    kNumModules = 4;
        public static final double kTrackWidth = Units.inchesToMeters(17.75);  // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(19.25); // Distance between front and back wheels

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kBackRightDriveMotorPort = 7;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kBackRightTurningMotorPort = 8;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad  = 2.780 ;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 4.0725;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad   = 0.785 ;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad  = 1.5254;


        // Max Tele Drive Speeds
        public static final double kTeleMaxSpeedMetersPerSecond = ModuleConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleMaxAngularSpeedRadiansPerSecond = ModuleConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;

        // Max Tele Drive Accels
        public static final double kTeleMaxAccelerationMetersPerSecondSquared = kTeleMaxSpeedMetersPerSecond * 6;
        public static final double kTeleMaxAngularAccelerationRadiansPerSecondSquared = kTeleMaxAngularSpeedRadiansPerSecond * 4 ;
   
    }

    public static final class AutoConstants {
        // Max Tele Drive Speeds
        public static final double kAutoMaxSpeedMetersPerSecond = ModuleConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kAutoMaxAngularSpeedRadiansPerSecond = ModuleConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 3;

        public static final double kAutoMaxAccelerationMetersPerSecondSquared = kAutoMaxSpeedMetersPerSecond * 4;
        public static final double kAutoMaxAngularAccelerationRadiansPerSecondSquared = kAutoMaxAngularSpeedRadiansPerSecond * 4 ;

        public static final double kPXController = 3.0;
        public static final double kPYController = 1.8;
        public static final double kPHeadingController = 6;
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
        public static final int kDriverResetRobotHeadingButtonIdx = 8;  // PSM

        public static final double kDeadband = 0.05;
    }
}
