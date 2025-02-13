package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class EndEffector extends SubsystemBase 
{
    private static EndEffector instance;
    private TalonFX motor;

    private Canandcolor frontCanandcolor;
    private Canandcolor backCanandcolor;
    
    private EndEffector ()
    {
        motor = new TalonFX(RobotMap.EndEffector.ID, RobotMap.CAN_CHAIN);
        config();

        frontCanandcolor = new Canandcolor(RobotMap.EndEffector.CORAL_CANANDCOLOR_ID);
        backCanandcolor = new Canandcolor(RobotMap.EndEffector.ALGAE_CANANDCOLOR_ID);
    }

    private void config ()
    {
        motor.clearStickyFaults();

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = RobotMap.EndEffector.INVERTED;
        config.Slot0.kP = RobotMap.EndEffector.kP;
        config.Slot0.kI = RobotMap.EndEffector.kI;
        config.Slot0.kD = RobotMap.EndEffector.kD;

        config.Voltage.PeakForwardVoltage = RobotMap.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -RobotMap.MAX_VOLTAGE;

        config.CurrentLimits.StatorCurrentLimit = RobotMap.EndEffector.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motor.getConfigurator().apply(config);
    }

    /**
     * @return rotations per second
     */
    public double getSpeed ()
    {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * @param speed rotations per second
     */
    public void setSpeed (double speed)
    {
        motor.setControl(new VelocityVoltage(speed));
    }

    public boolean hasCoral ()
    {
        return frontCanandcolor.getProximity() < RobotMap.EndEffector.PROXIMITY_LIMIT
            && backCanandcolor.getProximity() < RobotMap.EndEffector.PROXIMITY_LIMIT;
    }

    public boolean hasAlgae ()
    {
        return frontCanandcolor.getProximity() < RobotMap.EndEffector.PROXIMITY_LIMIT;
    }

    public static EndEffector getInstance ()
    {
        if (instance == null) instance = new EndEffector();
        return instance;
    }
}
