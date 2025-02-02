package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotMap;
import frc.robot.util.Telemetry;

public class SwerveModule {
    //motors on the swerve modules
    private TalonFX translation;
    private TalonFX rotation;

    private CANcoder canCoder;

    private SwerveModuleState optimizedState;

    //swerve module id
    public int ID;

    private static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(RobotMap.SwerveModule.TRANSLATION_kS, RobotMap.SwerveModule.TRANSLATION_kV, RobotMap.SwerveModule.TRANSLATION_kA);
    private VoltageOut driveCharacterizationControl = new VoltageOut(0);

    private VelocityVoltage driveVelocity;

    private PositionVoltage anglePosition;
    
    public SwerveModule(int id) {
        ID = id;

        driveVelocity = new VelocityVoltage(0);
        anglePosition = new PositionVoltage(0);

        canCoder = new CANcoder(RobotMap.SwerveModule.CAN_CODER_ID[id], RobotMap.CAN_CHAIN);
        configCANcoder();
        
        rotation = new TalonFX(RobotMap.SwerveModule.ROTATION_IDS[id], RobotMap.CAN_CHAIN);
        configRotation();
        setAbsolutePosition();

        translation = new TalonFX(RobotMap.SwerveModule.TRANSLATION_IDS[id], RobotMap.CAN_CHAIN);
        configTranslation();
        zeroTranslation();
    }   

    private void configTranslation() {
        TalonFXConfiguration transConfig = new TalonFXConfiguration();

        transConfig.Voltage.PeakForwardVoltage = RobotMap.MAX_VOLTAGE;
        transConfig.Voltage.PeakReverseVoltage = -RobotMap.MAX_VOLTAGE;

        transConfig.MotorOutput.Inverted = RobotMap.SwerveModule.TRANSLATION_INVERTS[ID];
        transConfig.Feedback.SensorToMechanismRatio = RobotMap.SwerveModule.TRANSLATION_GEAR_RATIO;
        transConfig.ClosedLoopGeneral.ContinuousWrap = true;
        transConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        transConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        transConfig.CurrentLimits.SupplyCurrentLimit = RobotMap.SwerveModule.TRANS_CURRENT_LIMIT;
        transConfig.CurrentLimits.SupplyCurrentLowerLimit = RobotMap.SwerveModule.TRANS_LOWER_CURRENT_LIMIT;
        transConfig.CurrentLimits.SupplyCurrentLowerTime = RobotMap.SwerveModule.TRANS_LOWER_LIMIT_TIME;

        transConfig.Slot0.kP = RobotMap.SwerveModule.TRANSLATION_kP;
        transConfig.Slot0.kI = RobotMap.SwerveModule.TRANSLATION_kI;
        transConfig.Slot0.kD = RobotMap.SwerveModule.TRANSLATION_kD;

        translation.getVelocity().setUpdateFrequency(250);
        translation.getPosition().setUpdateFrequency(250);
        translation.getConfigurator().apply(transConfig);
    }

    private void configRotation() {
        TalonFXConfiguration rotConfig = new TalonFXConfiguration();
        rotConfig.Voltage.PeakForwardVoltage = RobotMap.MAX_VOLTAGE;
        rotConfig.Voltage.PeakReverseVoltage = -RobotMap.MAX_VOLTAGE;

        rotConfig.MotorOutput.Inverted = RobotMap.SwerveModule.ROTATION_INVERTS[ID];
        rotConfig.Feedback.SensorToMechanismRatio = RobotMap.SwerveModule.ROTATION_GEAR_RATIO;
        rotConfig.ClosedLoopGeneral.ContinuousWrap = true;
        rotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rotConfig.CurrentLimits.SupplyCurrentLimit = RobotMap.SwerveModule.ROT_CURRENT_LIMIT;
        rotConfig.CurrentLimits.SupplyCurrentLowerLimit = RobotMap.SwerveModule.ROT_LOWER_CURRENT_LIMIT;
        rotConfig.CurrentLimits.SupplyCurrentLowerTime = RobotMap.SwerveModule.ROT_LOWER_LIMIT_TIME;

        rotConfig.Slot0.kP = RobotMap.SwerveModule.ROTATION_kP;
        rotConfig.Slot0.kI = RobotMap.SwerveModule.ROTATION_kI;
        rotConfig.Slot0.kD = RobotMap.SwerveModule.ROTATION_kD;

        rotation.getPosition().setUpdateFrequency(250);

        rotation.getConfigurator().apply(rotConfig);
    }

    private void configCANcoder() {
        canCoder.clearStickyFaults();

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset = -RobotMap.SwerveModule.CAN_CODER_OFFSETS[ID]; // offset is ADDED, so -offset

        canCoder.getConfigurator().apply(canCoderConfig);
    }
    /**
     * Sets translation and rotation motors to move to new state
     * @param state new state 
     */
    public void setAngleAndDrive(SwerveModuleState state) {
        state = optimize(state);

        rotation.setControl(anglePosition.withPosition(state.angle.getRotations()));
        
        driveVelocity.Velocity = state.speedMetersPerSecond/RobotMap.SwerveModule.TRANS_ROT_TO_METERS;
        driveVelocity.FeedForward = feedforward.calculate(state.speedMetersPerSecond);
        translation.setControl(driveVelocity);

        Telemetry.putModule(ID, "Desired Velocity", state.speedMetersPerSecond);
        Telemetry.putModule(ID, "Current Velocity", getSpeed());

        Telemetry.putModule(ID, "Desired Angle", state.angle.getDegrees());
        Telemetry.putModule(ID, "Current Angle", getAngle());
    }
    /*
     * adjusts the angle of a swerve module state 
     */
    public SwerveModuleState optimize (SwerveModuleState desiredState) {
    
        optimizedState = desiredState;

       double currentAngle = Math.toRadians(getAngle());
       double targetAngle = Math.IEEEremainder(desiredState.angle.getRadians(), Math.PI * 2);
       double remainder = currentAngle % (Math.PI * 2);
        var adjusted = targetAngle + currentAngle - remainder;

        var speed = desiredState.speedMetersPerSecond;
        // SmartDashboard.putNumber(swerveIDToName(ID) + "Desired Translation Speed", speed);
        
        if(adjusted - currentAngle > Math.PI) {
            adjusted -= Math.PI * 2;
        }
        if (adjusted - currentAngle < -Math.PI) {
            adjusted += Math.PI * 2;
        }
        if (adjusted - currentAngle > Math.PI / 2) {
            adjusted -= Math.PI;
            speed *= -1;
        } else if (adjusted - currentAngle < -Math.PI / 2) {
            adjusted += Math.PI;
            speed *= -1;
        }
        return new SwerveModuleState(speed, Rotation2d.fromRadians(adjusted));
        
    }
    /*
     * resets the swerve module
     */
    private void setAbsolutePosition() {
        canCoder.getAbsolutePosition().refresh();
        double position = canCoder.getAbsolutePosition().getValueAsDouble();// rotations
        // SmartDashboard.putNumber("CANCoder Pos (Raw) " + ID, canCoder.getAbsolutePosition().getValue());
        // SmartDashboard.putNumber("CANCoder Pos + offset " + ID, position);
        rotation.setPosition(position); // rotations
    }

    public void zeroTranslation() {
        translation.getConfigurator().setPosition(0); // rotations
    }
    
    /*
     * returns the angle of the rotation motor 
     */
    public double getAngle() {
        return rotation.getPosition().getValueAsDouble() * RobotMap.SwerveModule.ROT_ROT_TO_ANGLE;
    }

    /*
     * return speed of translation motor 
     */ 
    public double getSpeed() {
        return translation.getVelocity().getValueAsDouble() * RobotMap.SwerveModule.TRANS_ROT_TO_METERS;
    }

    /**
     * returns the voltage of the translation motor
     * @return voltage of translation motor
     */
    public double getTranslationVoltage() {
        return translation.getMotorVoltage().getValueAsDouble();
    }

    /**
     * returns the voltage of the rotation motor
     * @return voltage of rotation motor
     */
    public double getRotationVoltage() {
        return rotation.getMotorVoltage().getValueAsDouble();
    }

    /*
     * returns position of translation motor
     */
    public double getWheelPosition() {
        return translation.getPosition().getValueAsDouble() * RobotMap.SwerveModule.TRANS_ROT_TO_METERS;
    }

    //returns position and angle
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getWheelPosition(), Rotation2d.fromDegrees(getAngle()));
    }

    public SwerveModuleState getOptimizedModuleState() {
        return optimizedState;
    }

    //returns speed and angle
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getAngle()));
    }

    public void runCharacterization(double volts) {
        rotation.setControl(anglePosition.withPosition(new Rotation2d().getRotations()));
        this.runDriveCharacterization(volts);
    }

    public void runDriveCharacterization(double volts) {
        translation.setControl(driveCharacterizationControl.withOutput(volts));
    }
    
    //name of module on smart dashbaord 
    public static String swerveIDToName(int swerveID) {
        String output = "";
        if (swerveID < 2) output += "Front ";
        else output += "Back ";
        if (swerveID % 2 == 0) output += "Left";
        else output += "Right";
        return output;
    }
}