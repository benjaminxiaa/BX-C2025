
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import harkerrobolib.motors.HSTalonFX;
import harkerrobolib.util.MathUtil;
import harkerrobolib.util.PIDConfig;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    private HSTalonFX master;
    private HSTalonFX follower;

    private DigitalInput limitSwitch;

    private static double desiredPosition;

    private static double operatorDesiredPosition;

    private static boolean isManual;

    private Elevator() {
        // master = new TalonFX(Constants.Elevator.MASTER_ID, Constants.CAN_CHAIN);
        master = new HSTalonFX(Constants.Elevator.MASTER_ID,
                HSTalonFX.makeDefaultConfig()
                        .setInverted(Constants.Elevator.MASTER_INVERTED)
                        .setStatorCurrentLimit(Constants.Elevator.STATOR_CURRENT_LIMIT)
                        .setSupplyCurrentLimit(Constants.Elevator.SUPPLY_CURRENT_LIMIT)
                        .setBrakeMode()
                        .setPIDConfig(0,
                                new PIDConfig(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD, 0,
                                        Constants.Elevator.kV, 0, Constants.Elevator.kG))
                        .setForwardSoftLimitThreshold(Constants.Elevator.FORWARD_SOFT_LIMIT)
                        .setReverseSoftLimitThreshold(Constants.Elevator.REVERSE_SOFT_LIMIT)
                        .setSensorToMechanismRatio(Constants.Elevator.ELEVATOR_GEAR_RATIO));

        follower = new HSTalonFX(Constants.Elevator.FOLLOWER_ID, master, false,
                HSTalonFX.makeDefaultConfig().setInverted(Constants.Elevator.FOLLOWER_INVERTED));

        limitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_ID);

        desiredPosition = 0.0;

        operatorDesiredPosition = 0.0;
    }

    private final SysIdRoutine _sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(2.0).per(Second), // Use default ramp rate (1 V/s)
                    Volts.of(2.0), // Use default step voltage (7 V)
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysId_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> Elevator.getInstance().setVoltage(output.in(Volts)), null, this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return _sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return _sysId.dynamic(direction);
    }

    /**
     * @param desired rotations
     */
    public boolean checkExtend(double desired) {
        return Math.abs(desired - getPosition()) < Constants.Elevator.MAX_ERROR;
    }

    /**
     * @return rotations
     */
    public double getPosition() {
        return master.getSensorPosition();
    }

    /**
     * @return rotations per second
     */
    public double getVelocity() {
        return master.getSensorVelocity();
    }

    /**
     * @param power range [-1, 1]
     */
    public void setElevatorPower(double power) {
        master.setControl(new DutyCycleOut(power));
    }

    public void setVoltage(double power) {
        master.setControl(new VoltageOut(power));
    }

    public void setSensorPosition(double position) {
        master.setSensorPosition(position);
    }

    public void setDesiredPosition(double position) {
        desiredPosition = position;
    }

    public double getDesiredPosition() {
        return desiredPosition;
    }

    public void setOperatorDesiredPosition(double position) {
        operatorDesiredPosition = position;
    }

    public double getOperatorDesiredPosition() {
        return operatorDesiredPosition;
    }

    public void setManual(boolean manual) {
        isManual = manual;
    }

    public boolean isManual() {
        return isManual;
    }

    public void moveToPosition() {
        master.setControl(new PositionVoltage(desiredPosition));
    }

    public void resetEncoders() {
        master.setSensorPosition(0);
        follower.setSensorPosition(0);
    }

    public boolean isLimitHit() {
        return !limitSwitch.get();
    }

    public boolean isStalling() {
        return master.getStatorCurrent() >= Constants.Elevator.ELEVATOR_STALLING_CURRENT;
    }

    public boolean atDesired() {
        return MathUtil.compareSetpoint(getPosition(), getDesiredPosition(), Constants.Elevator.MAX_ERROR);
    }

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }
        return instance;
    }
}