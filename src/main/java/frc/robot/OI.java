package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.XboxGamepad;

public class OI {
    private static OI instance;
    private XboxGamepad driver;

    private OI() {
        driver = new XboxGamepad(RobotMap.OI.DRIVER_ID);
        initBindings();
    }

    public XboxGamepad getDriver() {
        return driver;
    }

    private void initBindings() {
        driver.getButtonA().onTrue(new InstantCommand(() -> {
            Drivetrain.getInstance().toggleRobotCentric();
        }));

        // DURING TESTING: ALIGN ROBOT BOTTOM RIGHT CORNER WITH CORNER OF SUBWOOFER, WITH SHOOTER FACING SPEAKER
        driver.getButtonB().onTrue(new InstantCommand( () -> Drivetrain.getInstance().setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
    }

    public static OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }
    
}
