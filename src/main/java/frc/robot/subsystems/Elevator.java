
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase 
{
    private static Elevator instance;

    private TalonFX master;
    private TalonFX follower;

    private DigitalInput limitSwitch;

    private Elevator() 
    {
        master = new TalonFX(RobotMap.Elevator.MASTER_ID, RobotMap.CAN_CHAIN);

        follower = new TalonFX(RobotMap.Elevator.FOLLOWER_ID, RobotMap.CAN_CHAIN);

        master.clearStickyFaults();
        follower.clearStickyFaults();
        config();

        limitSwitch = new DigitalInput(RobotMap.Elevator.LIMIT_SWITCH_ID);

    }

    private void config() 
    {

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.Inverted = RobotMap.Elevator.MASTER_INVERTED;

        masterConfig.Slot0.kP = RobotMap.Elevator.kP;
        masterConfig.Slot0.kG = RobotMap.Elevator.kG;

        masterConfig.Voltage.PeakForwardVoltage = RobotMap.MAX_VOLTAGE;
        masterConfig.Voltage.PeakReverseVoltage = -RobotMap.MAX_VOLTAGE;

        masterConfig.CurrentLimits.StatorCurrentLimit = RobotMap.Elevator.STATOR_CURRENT_LIMIT;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = RobotMap.Elevator.FORWARD_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RobotMap.Elevator.REVERSE_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        masterConfig.MotionMagic.MotionMagicAcceleration = RobotMap.Elevator.CRUISE_ACCELERATION;
        masterConfig.MotionMagic.MotionMagicCruiseVelocity = RobotMap.Elevator.CRUISE_VELOCITY;

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        master.getConfigurator().apply(masterConfig);

        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        followerConfig.MotorOutput.Inverted = RobotMap.Elevator.FOLLOWER_INVERTED;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        follower.getConfigurator().apply(followerConfig);

        follower.setControl(new Follower(RobotMap.Elevator.MASTER_ID, false));
    }

    /**
     * @param desired rotations
     */
    public void moveToPosition(double desired) 
    {
        master.setControl(new MotionMagicVoltage(desired));
    }

    /**
     * @param desired rotations
     */
    public boolean checkExtend(double desired) 
    {
        return Math.abs(desired - getPosition()) < RobotMap.Elevator.MAX_ERROR;
    }

    /**
     * @return rotations
     */
    public double getPosition() 
    {
        return master.getPosition().getValueAsDouble();
    }

    /**
     * @param power range [-1, 1]
     */
    public void setExtensionPower(double power) 
    {
        master.setControl(new DutyCycleOut(power));
    }

    public void resetEncoders() 
    {
        master.getConfigurator().setPosition(0);
        follower.getConfigurator().setPosition(0);
    }

    public boolean extensionStop() 
    {
        return !limitSwitch.get();
    }

    public boolean isFarExtended() 
    {
        return getPosition() > RobotMap.Elevator.FAR_EXTENDED_DISTANCE;
    }

    @Override
    public void periodic() 
    {
    }

    public static Elevator getInstance() 
    {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }

    @Override
    public void initSendable(SendableBuilder builder) 
    {
        builder.setSmartDashboardType("Elevator");
        builder.setActuator(true);
        builder.setSafeState(() -> setExtensionPower(0));
        builder.addDoubleProperty("Position", this::getPosition, this::moveToPosition);
    }
}
