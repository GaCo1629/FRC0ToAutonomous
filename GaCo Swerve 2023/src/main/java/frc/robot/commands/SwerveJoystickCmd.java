package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdLeftFunction, turningSpdRightFunction;
    private final Supplier<Boolean> fieldOrientedFunction, goToTargetFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private Joystick driverJoystick, copilotJoystick;
    
    ProfiledPIDController xController;
    ProfiledPIDController yController;
    ProfiledPIDController headingController;
    ProfiledPIDController headingLockController;
    boolean PIDRunning = false;
    boolean headingLocked = false;
    double  headingSetpoint = 0;   
   
    PhotonCamera camera = new PhotonCamera(VisionConstants.cameraName);

    public SwerveJoystickCmd(SwerveSubsystem SwerveSubsystem, Joystick driverJoystick, Joystick copilotJoystick) {
        this.swerveSubsystem = SwerveSubsystem;
        this.driverJoystick = driverJoystick;
        this.copilotJoystick = copilotJoystick;
        xSpdFunction = () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis);
        ySpdFunction = () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis);
        turningSpdLeftFunction = () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxisLeft);
        turningSpdRightFunction = () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxisRight);
        fieldOrientedFunction = () -> driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx);
        goToTargetFunction = () -> driverJoystick.getRawButton(OIConstants.kDriverGoToTargetButtonIdx);

        xLimiter = new SlewRateLimiter(DriveConstants.kTeleMaxAccelerationMetersPerSecondSquared);
        yLimiter = new SlewRateLimiter(DriveConstants.kTeleMaxAccelerationMetersPerSecondSquared);
        turningLimiter = new SlewRateLimiter(DriveConstants.kTeleMaxAngularAccelerationRadiansPerSecondSquared);

        addRequirements(swerveSubsystem);
    
        xController = new ProfiledPIDController(AutoConstants.kPXController, 0, 0, 
                          new Constraints(AutoConstants.kAutoMaxSpeedMetersPerSecond,AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared));
        
        yController = new ProfiledPIDController(AutoConstants.kPYController, 0, 0, 
                          new Constraints(AutoConstants.kAutoMaxSpeedMetersPerSecond,AutoConstants.kAutoMaxAccelerationMetersPerSecondSquared));
        
        headingController = new ProfiledPIDController(AutoConstants.kPHeadingController, 0, 0, 
                          new Constraints(AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond,AutoConstants.kAutoMaxAngularAccelerationRadiansPerSecondSquared));
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        headingLockController = new ProfiledPIDController(AutoConstants.kPHeadingLockController, 0, 0,
                            new Constraints(AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond,AutoConstants.kAutoMaxAngularAccelerationRadiansPerSecondSquared));
        headingLockController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // Change this to match the name of your camera
    }

    @Override
    public void execute() {

        if (driverJoystick.getRawButtonPressed(OIConstants.kDriverResetRobotHeadingButtonIdx)){
                    swerveSubsystem.zeroHeading();
                    headingSetpoint = 0;
        }
    
        // 1. Get real-time joystick inputs
        double xJoystick = applyDeadband( -xSpdFunction.get(), OIConstants.kDeadband);
        double yJoystick = applyDeadband( -ySpdFunction.get(), OIConstants.kDeadband);
        double turningJoystick = applyDeadband(turningSpdLeftFunction.get() - turningSpdRightFunction.get(), OIConstants.kDeadband);

        // 2. Convert to real world units
        double xSpeed = xJoystick * DriveConstants.kTeleMaxSpeedMetersPerSecond;
        double ySpeed = yJoystick * DriveConstants.kTeleMaxSpeedMetersPerSecond;
        double turningSpeed = turningJoystick * DriveConstants.kTeleMaxAngularSpeedRadiansPerSecond;
       
        // 3. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        double targetToRobotX = 0;
        double targetToRobotY = 0;
        double targetToRobotT = 0;
        double targetToRobotR = 0;
        double targetToRobotB = 0;

        // find an apriltag and extract robot position
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
            if (targetToRobotT < 0) {
                targetToRobotT += Math.PI;
            }  else {
                targetToRobotT -= Math.PI;   
            }       
            targetToRobotT *= -1 ;
   
            // SmartDashboard.putNumber("TX", targetToRobotX);
            // SmartDashboard.putNumber("TY", targetToRobotY);
            // SmartDashboard.putNumber("TB", targetToRobotB);
            // SmartDashboard.putNumber("TR", targetToRobotR);
            SmartDashboard.putNumber("TT (r)", targetToRobotT);
            SmartDashboard.putNumber("TT (d)", Math.toDegrees(targetToRobotT));
        }
                
        // Either Track to AprilTag, or Use manual inputs
        if (goToTargetFunction.get() && result.hasTargets()) {
           /*  // Vision allignment mode

            // Load current position 
            if (!PIDRunning) {
                xController.reset(targetToRobotR);
                yController.reset(targetToRobotT);
                headingController.reset(targetToRobotB);
                PIDRunning = true;
            }
        
            turningSpeed = -headingController.calculate(targetToRobotB, 0);
            if (Math.abs(targetToRobotB) < 0.1) {
                turningSpeed = 0;
            } 

            xSpeed = -xController.calculate(targetToRobotR, AutoConstants.kTargetStandoff);
            if (Math.abs(targetToRobotR - AutoConstants.kTargetStandoff) < AutoConstants.kTargetStandoffDeadband) {
                xSpeed = 0;
            } 
            
            ySpeed = -yController.calculate(targetToRobotT, 0);
            if (Math.abs(targetToRobotT) < 0.075) {
                ySpeed = 0;
            } 
            */
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed); 
            
        } else {
            // Manual Driving
            PIDRunning = false;

            // Smooth Joystick Speeds
            xSpeed = xLimiter.calculate(xSpeed);
            ySpeed = yLimiter.calculate(ySpeed);
            turningSpeed = turningLimiter.calculate(turningSpeed);

            //Directional Buttons
            if (driverJoystick.getRawButtonPressed(1)) {
                headingSetpoint = Math.PI;
            } else if (driverJoystick.getRawButtonPressed(2)) {
                headingSetpoint = -Math.PI / 2;
            } else if (driverJoystick.getRawButtonPressed(3)) {
                headingSetpoint = Math.PI /2;
            } else if (driverJoystick.getRawButtonPressed(4)) {
                headingSetpoint = 0;
            }
            

            // Check Auto Heading

            if (Math.abs(turningSpeed) > 0.01) {
                headingLocked = false;
            } else if (!headingLocked && swerveSubsystem.isNotRotating()) {
                headingLocked = true;
                headingSetpoint = swerveSubsystem.getHeading();
            }

            if (headingLocked) {
                double headingError =  headingSetpoint - swerveSubsystem.getHeading();
                turningSpeed = -headingLockController.calculate(headingError, 0);
                if (turningSpeed > 5) {
                    turningSpeed = 5;
                } else if (turningSpeed < -5) {
                    turningSpeed = -5;
                }
                if (Math.abs(turningSpeed) < 0.1) {
                    turningSpeed = 0;
                } 
                SmartDashboard.putNumber("Heading Error", headingError);
            }

            SmartDashboard.putBoolean("Heading Locked", headingLocked);

            

            if (!fieldOrientedFunction.get()) {
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

    private double applyDeadband(double value, double deadband) {
        return  (Math.abs(value) < deadband) ? 0 : Math.signum(value) * (Math.abs(value) - deadband)  / (1 - deadband );
        
    }
}
