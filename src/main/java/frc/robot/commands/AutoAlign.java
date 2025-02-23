package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class AutoAlign extends Command {
    private final Drivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive;
    private int lockedTagId = -1;
    
    // Target values
    private static final double TARGET_TY = 15.0;
    
    // PID Controllers
    private final PIDController xController = new PIDController(0.03, 0, 0);
    private final PIDController rotationController = new PIDController(0.03, 0, 0);
    
    public AutoAlign(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
        
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xController.reset();
        rotationController.reset();
        lockedTagId = -1;
    }

    @Override
    public void execute() {
        // Check if we need to lock onto a tag
        if (lockedTagId == -1) {
            if (LimelightHelpers.getTV("limelight4")) {
                lockedTagId = (int)LimelightHelpers.getFiducialID("limelight4");
            } else {
                drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
                return;
            }
        }

        // Make sure we still see our locked tag
        if (!LimelightHelpers.getTV("limelight4") || 
            lockedTagId != (int)LimelightHelpers.getFiducialID("limelight4")) {
            drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        // Get target offsets
        double tx = LimelightHelpers.getTX("limelight4");
        double ty = LimelightHelpers.getTY("limelight4");
        
        // Calculate speeds - note ty is inverted for forward/back
        double xSpeed = 0;
        double rotSpeed = 0;

        // Rotation (keep tag centered horizontally)
        if (Math.abs(tx) > 1.0) {
            rotSpeed = -rotationController.calculate(tx, 0);
        }
        
        // Forward/back (maintain target distance)
        if (Math.abs(ty - TARGET_TY) > 1.0) {
            xSpeed = xController.calculate(ty, TARGET_TY);
        }
        
        // Clamp speeds
        xSpeed = clamp(xSpeed, -Constants.Drive.MAX_TRANSLATION_SPEED, Constants.Drive.MAX_TRANSLATION_SPEED);
        rotSpeed = clamp(rotSpeed, -Constants.Drive.MAX_ROTATION_SPEED, Constants.Drive.MAX_ROTATION_SPEED);
        
        // Apply control
        drivetrain.setControl(drive
            .withVelocityX(xSpeed)
            .withVelocityY(0)  // No strafe
            .withRotationalRate(rotSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        lockedTagId = -1;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}