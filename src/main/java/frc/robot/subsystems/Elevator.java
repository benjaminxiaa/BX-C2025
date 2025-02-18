
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase 
{
    private static Elevator instance;

    private TalonFX master;
    private TalonFX follower;

    private DigitalInput limitSwitch;

    private static double desiredPosition;

    private Elevator() 
    {
        master = new TalonFX(RobotMap.Elevator.MASTER_ID, RobotMap.CAN_CHAIN);

        follower = new TalonFX(RobotMap.Elevator.FOLLOWER_ID, RobotMap.CAN_CHAIN);

        config();

        limitSwitch = new DigitalInput(RobotMap.Elevator.LIMIT_SWITCH_ID);

        desiredPosition = 0.0;

    }

    private void config() 
    {
        master.clearStickyFaults();
        follower.clearStickyFaults();

        TalonFXConfiguration masterConfig = new TalonFXConfiguration();

        masterConfig.MotorOutput.Inverted = RobotMap.Elevator.MASTER_INVERTED;

        masterConfig.Feedback.SensorToMechanismRatio = RobotMap.Elevator.ELEVATOR_GEAR_RATIO;

        masterConfig.Slot0.kP = RobotMap.Elevator.kP;
        masterConfig.Slot0.kI = RobotMap.Elevator.kI;
        masterConfig.Slot0.kD = RobotMap.Elevator.kD;

        masterConfig.Slot0.kV = RobotMap.Elevator.kV;
        masterConfig.Slot0.kG = RobotMap.Elevator.kG;

        masterConfig.Voltage.PeakForwardVoltage = RobotMap.MAX_VOLTAGE;
        masterConfig.Voltage.PeakReverseVoltage = -RobotMap.MAX_VOLTAGE;

        masterConfig.CurrentLimits.StatorCurrentLimit = RobotMap.Elevator.STATOR_CURRENT_LIMIT;
        masterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        masterConfig.CurrentLimits.SupplyCurrentLimit = RobotMap.Elevator.SUPPLY_CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = RobotMap.Elevator.FORWARD_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = RobotMap.Elevator.REVERSE_SOFT_LIMIT;
        masterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        master.getConfigurator().apply(masterConfig);

        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        followerConfig.MotorOutput.Inverted = RobotMap.Elevator.FOLLOWER_INVERTED;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        follower.getConfigurator().apply(followerConfig);

        follower.setControl(new Follower(RobotMap.Elevator.MASTER_ID, false));
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
     * @return rotations per second
     */
    public double getVelocity()
    {
        return master.getVelocity().getValueAsDouble();
    }

    /**
     * @param power range [-1, 1]
     */
    public void setElevatorPower(double power) 
    {
        master.setControl(new DutyCycleOut(power));
    }

    public void setSensorPosition(double position)
    {
        master.getConfigurator().setPosition(0);
    }

    public void setDesiredPosition(double position)
    {
        desiredPosition = position;
    }

    public double getDesiredPosition()
    {
        return desiredPosition;
    }

    public void moveToPosition()
    {
        master.setControl(new PositionVoltage(desiredPosition));
    }

    public void resetEncoders() 
    {
        master.getConfigurator().setPosition(0);
        follower.getConfigurator().setPosition(0);
    }

    public boolean isLimitHit() 
    {
        return !limitSwitch.get();
    }

    public boolean isStalling()
    {
        return master.getStatorCurrent().getValueAsDouble() >= RobotMap.Elevator.ELEVATOR_STALLING_CURRENT;
    }

    public static Elevator getInstance() 
    {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
}
