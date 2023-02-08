package frc.robot.commands;

import java.lang.ModuleLayer.Controller;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
    
    ProfiledPIDController xController;
    ProfiledPIDController yController;
    ProfiledPIDController headingController;
    boolean       PIDRunning = false;
   
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
        this.xLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        this.turningLimiter = new SlewRateLimiter(AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
        addRequirements(swerveSubsystem);
        xController = new ProfiledPIDController(AutoConstants.kPXController, 0, 0, 
                          new Constraints(AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        
        yController = new ProfiledPIDController(AutoConstants.kPYController, 0, 0, 
                          new Constraints(AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        
        headingController = new ProfiledPIDController(AutoConstants.kPHeadingController, 0, 0, 
                          new Constraints(AutoConstants.kMaxAngularSpeedRadiansPerSecond,AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared));

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        
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

        // 3. Convert to real world units
        double xSpeed = xJoystick * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        double ySpeed = yJoystick * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        double turningSpeed = turningJoystick * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
       
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        double targetToRobotX = 0;
        double targetToRobotY = 0;
        double targetToRobotT = 0;
        double targetToRobotR = 0;
        double targetToRobotB = 0;

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

            if (!PIDRunning) {
                xController.reset(targetToRobotR);
                yController.reset(targetToRobotT);
                headingController.reset(targetToRobotB);
                PIDRunning = true;
            }
        
            double outputH = headingController.calculate(targetToRobotB, 0);
            if (Math.abs(targetToRobotB) < 0.1) {
                outputH = 0;
            } 

            double outputX = -xController.calculate(targetToRobotR, AutoConstants.kTargetStandoff);
            if (Math.abs(targetToRobotR - AutoConstants.kTargetStandoff) < AutoConstants.kTargetStandoffDeadband) {
                outputX = 0;
            } 
            
            double outputY = -yController.calculate(targetToRobotT, 0);
            if (Math.abs(targetToRobotT) < 0.075) {
                outputY = 0;
            } 
           
            xSpeed = outputX;
            ySpeed = outputY;
            turningSpeed = outputH;

            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        } else {


            PIDRunning = false;

            // Smooth Speeds
            xSpeed = xLimiter.calculate(xSpeed);
            ySpeed = yLimiter.calculate(ySpeed);
            turningSpeed = turningLimiter.calculate(turningSpeed);

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
        SmartDashboard.putNumber("Turn Speed", turningSpeed);

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
