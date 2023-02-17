package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.PhotonCameraWrapper;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);  
    private double gyro2FieldOffset = 0;
    private double gyro2FCDOffset = 0;
    
    private final PhotonCameraWrapper pcw = new PhotonCameraWrapper();

    private final SwerveDrivePoseEstimator odometer =  new SwerveDrivePoseEstimator(
                    DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions(), new Pose2d());
    
    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetGyroToZero();
            } catch (Exception e) {
            }
        }).start();
    }

    public double resetGyroToZero() {
        gyro.reset();
        if (DriverStation.getAlliance() == Alliance.Red){
            gyro2FieldOffset = 0.0;
            gyro2FCDOffset = Math.PI;
        } else {
            gyro2FieldOffset = Math.PI;  
            gyro2FCDOffset = 0.0; 
        }

        return getHeading();
    }

    public double getHeading() {
        return Math.IEEEremainder(Math.toRadians(-gyro.getAngle() + gyro2FieldOffset), Math.PI * 2);
    }

    public double getFCDHeading() {
        return Math.IEEEremainder(Math.toRadians(-gyro.getAngle() + gyro2FCDOffset), Math.PI * 2);
    }

    public boolean isNotRotating() {
        SmartDashboard.putNumber("Rotate rate", gyro.getRate());

        return (Math.abs(gyro.getRate()) < OIConstants.kNotRotating);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(getHeading());
    }

    public Rotation2d getFCDRotation2d() {
        return Rotation2d.fromRadians(getFCDHeading());
    }

    public Pose2d getPose() {
     return odometer.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);  // Phil's best guess.
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose(odometer.getEstimatedPosition()); 

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            odometer.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
        }

        SmartDashboard.putNumber("Robot Heading", Math.toDegrees(getHeading()));
        SmartDashboard.putNumber("Robot FCD Heading", Math.toDegrees(getFCDHeading()));
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
     
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
