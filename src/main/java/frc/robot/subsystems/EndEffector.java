package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class EndEffector extends SubsystemBase 
{
    private static EndEffector instance;
    private TalonFX motor;

    private DigitalInput forwardLimitSwitch;
    private DigitalInput backwardLimitSwitch;

    boolean intaking = false;
    
    private EndEffector ()
    {
        motor = new TalonFX(RobotMap.EndEffector.ID, RobotMap.CAN_CHAIN);
        config();

        forwardLimitSwitch = new DigitalInput(RobotMap.EndEffector.FORWARD_LIMIT_SWITCH_ID);
        backwardLimitSwitch = new DigitalInput(RobotMap.EndEffector.BACKWARD_LIMIT_SWITCH_ID);
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

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = RobotMap.EndEffector.FORWARD_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RobotMap.EndEffector.REVERSE_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motor.getConfigurator().apply(config);
    }

    /**
     * @return meters per second
     */
    public double getSpeed ()
    {
        return motor.getVelocity().getValueAsDouble() * RobotMap.EndEffector.ROT_TO_METERS;
    }

    /**
     * @param speed meters per second
     */
    public void setSpeed (double speed)
    {
        motor.setControl(new VelocityVoltage(speed / RobotMap.EndEffector.ROT_TO_METERS));
    }

    public boolean intakeStop ()
    {
        return forwardLimitSwitch.get() && !backwardLimitSwitch.get();
    }

    public boolean hasNoCoral ()
    {
        return !forwardLimitSwitch.get() && !backwardLimitSwitch.get();
    }

    @Override
    public void periodic() 
    {
    }

    public static EndEffector getInstance ()
    {
        if (instance == null) instance = new EndEffector();
        return instance;
    }
}
