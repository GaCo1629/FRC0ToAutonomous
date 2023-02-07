package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction, goToTargetFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    PhotonCamera camera = new PhotonCamera("OV5647");

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> goToTargetFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.goToTargetFunction = goToTargetFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Change this to match the name of your camera
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xJoystick = xSpdFunction.get();
        double yJoystick = ySpdFunction.get();
        double turningJoystick = turningSpdFunction.get();

        // 2. Apply deadband
        xJoystick = Math.abs(xJoystick) > OIConstants.kDeadband ? xJoystick : 0.0;
        yJoystick = Math.abs(yJoystick) > OIConstants.kDeadband ? yJoystick : 0.0;
        turningJoystick = Math.abs(turningJoystick) > OIConstants.kDeadband ? turningJoystick : 0.0;

        // 3. Make the driving smoother
        double xSpeed = xLimiter.calculate(xJoystick) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        double ySpeed = yLimiter.calculate(yJoystick) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        double turningSpeed = turningLimiter.calculate(turningJoystick)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        double targetToRobotX = 0;
        double targetToRobotY = 0;
        double targetToRobotT = 0;
        double targetToRobotR = 0;
        double targetToRobotB = 0;

        ProfiledPIDController xController = new ProfiledPIDController(
                AutoConstants.kPXController, 0, 0, AutoConstants.kXControllerConstraints);
        
        ProfiledPIDController yController = new ProfiledPIDController(
                AutoConstants.kPYController, 0, 0, AutoConstants.kXControllerConstraints);
        yController.setTolerance(0.08);
        
        ProfiledPIDController headingController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        // find an apriltag and extract robot
        // Calculate and display Apriltag data here so we always have it.  Note:  May need to move to own subsystem.
        var result = camera.getLatestResult();
        if (result.hasTargets()) {

            Transform3d robotPosition = result.getBestTarget().getBestCameraToTarget();
            targetToRobotX = robotPosition.getX();
            targetToRobotY = robotPosition.getY();
            targetToRobotT = robotPosition.getRotation().toRotation2d().getRadians();

            // Calculate Range and Bearing to target
            targetToRobotR = Math.hypot(targetToRobotX, targetToRobotY);
            targetToRobotB = Math.atan2(targetToRobotY, targetToRobotX);

            // Invert direction of theta.
            if (targetToRobotT < 0)
                targetToRobotT += Math.PI;
            else
                targetToRobotT -= Math.PI;

            SmartDashboard.putNumber("TX", targetToRobotX);
            SmartDashboard.putNumber("TY", targetToRobotY);
            SmartDashboard.putNumber("TT", targetToRobotT);

            SmartDashboard.putNumber("TR", targetToRobotR);
            SmartDashboard.putNumber("TB", targetToRobotB);
        }

        // Either Track to AprilTag, or Use manual inputs
        if (!goToTargetFunction.get() && result.hasTargets()) {
            // Vision allignment mode
        
            double outputH = headingController.calculate(targetToRobotB, 0);
            double outputX = -xController.calculate(targetToRobotR, AutoConstants.kTargetStandoff);
            double outputY = -yController.calculate(targetToRobotT, 0);

            SmartDashboard.putNumber("TX P SP", xController.getSetpoint().position);
            SmartDashboard.putNumber("TX V SP", xController.getSetpoint().velocity);

            xSpeed = outputX;
            ySpeed = outputY;
            turningSpeed = outputH;
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        } else {
            xController.reset(AutoConstants.kTargetStandoff);
            yController.reset(0);
            headingController.reset(0);
            if (fieldOrientedFunction.get()) {
                // Relative to field
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            } else {
                // Relative to robot
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            }
        }

        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putNumber("Y Speed", ySpeed);
        SmartDashboard.putString("T Speed", String.format("%.2f", turningSpeed));

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
