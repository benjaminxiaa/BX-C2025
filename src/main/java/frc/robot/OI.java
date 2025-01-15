package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    private static OI instance;
    private XboxController driver;

    private OI() {
        driver = new XboxController(RobotMap.OI.DRIVER_ID);
    }

    public XboxController getDriver() {
        return driver;
    }

    public static OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }
    
}
