package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import harkerrobolib.motors.HSTalonFX;
import harkerrobolib.util.PIDConfig;

public class EndEffector extends SubsystemBase {
    private static EndEffector instance;
    private HSTalonFX motor;

    private Canandcolor frontCanandcolor;
    private Canandcolor backCanandcolor;

    private EndEffector() {
        motor = new HSTalonFX(Constants.EndEffector.ID, HSTalonFX.makeDefaultConfig()
                .setInverted(true)
                .setPIDConfig(0,
                        new PIDConfig(Constants.EndEffector.kP, Constants.EndEffector.kI, Constants.EndEffector.kD))
                .setStatorCurrentLimit(Constants.EndEffector.STATOR_CURRENT_LIMIT));

        frontCanandcolor = new Canandcolor(Constants.EndEffector.FRONT_CANANDCOLOR_ID);
        backCanandcolor = new Canandcolor(Constants.EndEffector.BACK_CANANDCOLOR_ID);
    }

    /**
     * @return rotations per second
     */
    public double getSpeed() {
        return motor.getSensorVelocity();
    }

    /**
     * @param speed rotations per second
     */
    public void setSpeed(double speed) {
        // motor.setControl(new VelocityVoltage(speed));
        motor.setControl(new DutyCycleOut(speed));
    }

    public boolean isBackTriggered() {
        return backCanandcolor.getProximity() < Constants.EndEffector.PROXIMITY_LIMIT_BACK;
    }

    public boolean isFrontTriggered() {
        return frontCanandcolor.getProximity() < Constants.EndEffector.PROXIMITY_LIMIT_FRONT;
    }

    public static EndEffector getInstance() {
        if (instance == null)
            instance = new EndEffector();
        return instance;
    }
}