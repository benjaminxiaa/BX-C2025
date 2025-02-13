package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.EE.IntakeAlgae;
import frc.robot.commands.EE.IntakeCoral;
import frc.robot.commands.EE.Outtake;
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

        driver.getButtonB().onTrue(new InstantCommand( () -> Drivetrain.getInstance().setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));

        // combine into one in the future?
        driver.getButtonX().onTrue(new IntakeAlgae());
        driver.getButtonY().onTrue(new IntakeCoral());

        driver.getLeftBumper().onTrue(new Outtake());
    }

    public static OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }
    
}
