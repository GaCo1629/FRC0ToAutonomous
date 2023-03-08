package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private final Supplier<Double> xSpdFunction, ySpdFunction, turnSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction, goToTargetFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private Joystick driverJoystick, copilotJoystick;
    
    ProfiledPIDController xController;
    ProfiledPIDController yController;
    ProfiledPIDController headingController;
    ProfiledPIDController headingLockController;
    boolean PIDRunning = false;
    boolean headingLocked = false;
    double  currentHeading = 0;
    double  headingSetpoint = 0;   
    Alliance ourAlliance = Alliance.Red;

       
    PhotonCamera camera = new PhotonCamera(VisionConstants.cameraName);

    public SwerveJoystickCmd(SwerveSubsystem SwerveSubsystem, Joystick driverJoystick, Joystick copilotJoystick) {
        this.swerveSubsystem = SwerveSubsystem;
        this.driverJoystick = driverJoystick;
        this.copilotJoystick = copilotJoystick;
        xSpdFunction = () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis);
        ySpdFunction = () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis);
        turnSpdFunction = () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis);
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
        // Set the current heeding as the 
        currentHeading = swerveSubsystem.getHeading();
        lockCurrentHeading();

        ourAlliance = DriverStation.getAlliance();
    }

    @Override
    public void execute() {

        currentHeading = swerveSubsystem.getHeading();  // just read once so gyro is not read repeatedly

        if (driverJoystick.getRawButtonPressed(OIConstants.kDriverResetRobotHeadingButtonIdx)){
            currentHeading = swerveSubsystem.resetGyroToZero();;
            lockCurrentHeading();
        }
   
        // 1. Get real-time joystick inputs
        double xJoystick = applyDeadband( -xSpdFunction.get(), OIConstants.kDeadband);
        double yJoystick = applyDeadband( -ySpdFunction.get(), OIConstants.kDeadband);
        double turnJoystick = applyDeadband( -turnSpdFunction.get(), OIConstants.kDeadband);

        // 2. Convert to real world units
        double xSpeed = xJoystick * DriveConstants.kTeleMaxSpeedMetersPerSecond;
        double ySpeed = yJoystick * DriveConstants.kTeleMaxSpeedMetersPerSecond;
        double turnSpeed = turnJoystick * DriveConstants.kTeleMaxAngularSpeedRadiansPerSecond;
       
        // 3. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;

        double targetToRobotX = 0;
        double targetToRobotY = 0;
        double targetToRobotT = 0;

        double targetToRobotR ;
        double targetToRobotB ;

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
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed); 
            
        } else {
            // Manual Driving
            PIDRunning = false;

            // Smooth Joystick Speeds
            xSpeed = xLimiter.calculate(xSpeed);
            ySpeed = yLimiter.calculate(ySpeed);
            turnSpeed = turningLimiter.calculate(turnSpeed);

            //Directional Buttons
            if (DriverStation.getAlliance() == Alliance.Red) {
                if (driverJoystick.getRawButtonPressed(1)) {
                    newHeadingSetpoint(0);
                } else if (driverJoystick.getRawButtonPressed(2)) {
                    newHeadingSetpoint(Math.PI / 2);
                } else if (driverJoystick.getRawButtonPressed(3)) {
                    newHeadingSetpoint(-Math.PI / 2);
                } else if (driverJoystick.getRawButtonPressed(4)) {
                    newHeadingSetpoint(Math.PI);
                }
            } else {
                if (driverJoystick.getRawButtonPressed(1)) {
                    newHeadingSetpoint(Math.PI);
                } else if (driverJoystick.getRawButtonPressed(2)) {
                    newHeadingSetpoint(-Math.PI / 2);
                } else if (driverJoystick.getRawButtonPressed(3)) {
                    newHeadingSetpoint(Math.PI / 2);
                } else if (driverJoystick.getRawButtonPressed(4)) {
                    newHeadingSetpoint(0);
                }
            }
                

            // Check Auto Heading
            if (Math.abs(turnSpeed) > 0.01) {
                headingLocked = false;
            } else if (!headingLocked && swerveSubsystem.isNotRotating()) {
                headingLocked = true;
                lockCurrentHeading(); 
            }

            if (headingLocked) {

                turnSpeed = headingLockController.calculate(currentHeading, headingSetpoint);
                if (Math.abs(turnSpeed) < 0.1) {
                    turnSpeed = 0;
                } 
            }

            SmartDashboard.putBoolean("Heading Locked", headingLocked);

            

            if (!fieldOrientedFunction.get()) {
                // Relative to field
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, turnSpeed, swerveSubsystem.getFCDRotation2d());
            } else {
                // Relative to robot
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
            }
        }

         SmartDashboard.putNumber("X Speed", xSpeed);
         SmartDashboard.putNumber("Y Speed", ySpeed);
         SmartDashboard.putNumber("Turn Speed", turnSpeed);

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

    public void newHeadingSetpoint(double newSetpoint) {
        headingSetpoint = newSetpoint;
        headingLockController.reset(currentHeading);
    }

    public void lockCurrentHeading() {
        newHeadingSetpoint(currentHeading);
    }

    private double applyDeadband(double value, double deadband) {
        return  (Math.abs(value) < deadband) ? 0 : Math.signum(value) * (Math.abs(value) - deadband)  / (1 - deadband );
        
    }
}
