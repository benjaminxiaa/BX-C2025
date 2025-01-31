package frc.robot;

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
        driver.getButtonStart().onTrue(new InstantCommand(() -> {
            Drivetrain.getInstance().toggleRobotCentric();
        }));
    }

    public static OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }
    
}
