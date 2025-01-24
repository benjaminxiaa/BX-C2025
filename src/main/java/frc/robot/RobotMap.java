package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

public class RobotMap 
{

    // Global Robot Constants
    public static final double MAX_VOLTAGE = 12;
    public static final double ROBOT_LOOP = 0.02;
    public static final String CAN_CHAIN = "Drivetrain";
    

    public static final class OI 
    {
        public static final double JOYSTICK_DEADBAND = 0.15;
        public static final double TRIGGER_DEADBAND = 0.1;
      
        public static final int DRIVER_ID = 0;
        public static final int OPERATOR_ID = 1;
    }

    public static final class PID 
    {
        public static final int PID_PRIMARY = 0;
        public static final int PID_AUXILIARY = 1;
      
        public static final int SLOT_INDEX = 0;
    }

    public static final class SwerveModule 
    {
        // id of translation motors
        // FL, FR, BL, BR
        public static final int[] TRANSLATION_IDS = {13, 3, 9, 6};

        // translation motors inverted
        public static final InvertedValue[] TRANSLATION_INVERTS = new InvertedValue[]{InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive};

        // ids for rotation motors
        public static final int[] ROTATION_IDS = {11, 1, 8, 4};

        // rotation motors inverted
        public static final InvertedValue[] ROTATION_INVERTS = {InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive};

        // cancoder ids
        public static final int[] CAN_CODER_ID = {12, 2, 10, 5};

        // offsets of cancoders of each swerve module (in rotations)
        public static final double[] CAN_CODER_OFFSETS = new double[]{-0.206055, 0.169922, -0.489746, -0.424316};
        // current limit constants for translation motors
        public static final double TRANS_CURRENT_LIMIT = 40;
        public static final double TRANS_THRESHOLD_CURRENT = 55;
        public static final double TRANS_THRESHOLD_TIME= 0.1;

        // current limit constants for rotation motors
        public static final double ROT_CURRENT_LIMIT = 25;
        public static final double ROT_THRESHOLD_CURRENT = 40;
        public static final double ROT_THRESHOLD_TIME = 0.1;

        // gear ratios
        public static final double TRANSLATION_GEAR_RATIO = 6.12;
        public static final double ROTATION_GEAR_RATIO = 150.0 / 7.0; 
        // diameter of the wheel
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0); // meters

        // conversions from rotations -- NO GEAR RATIOS!!!
        public static final double TRANS_ROT_TO_METERS = WHEEL_DIAMETER * Math.PI; // rotations to meters
        public static final double ROT_ROT_TO_ANGLE = 360.0; // rotations to degrees

        // rotation kP
        public static final double ROTATION_kP = 50; // TODO
        public static final double ROTATION_kI = 0;
        public static final double ROTATION_kD = 0;

        // Translation FF Values
        public static final double TRANSLATION_kS = 0.13561; // TODO
        public static final double TRANSLATION_kV = 1.9051; // TODO
        public static final double TRANSLATION_kA = 1.5737; // TODO

        // pid
        public static final double TRANSLATION_kP = 1.7; // TODO
        public static final double TRANSLATION_kI = 0.5; // TODO
        public static final double TRANSLATION_kD = 0.00;  // TODO
    }

    public static final class Drivetrain 
    {
        // Pigeon ID
        public static final int PIGEON_ID = 7;

        public static final double PIGEON_kP = 0.00012; // TODO

        public static final double MIN_OUTPUT = 0.05;
        
        // Robot Dimensions
        public static final double ROBOT_LENGTH = Units.inchesToMeters(28.5);
        public static final double ROBOT_WIDTH = Units.inchesToMeters(28.5);

        public static final double MAX_DRIVING_SPEED = 5.0; // m/s //TODO
        public static final double EXTENDED_MAX_DRIVING_SPEED_MULTIPLIER = 0.4;
        public static final double MAX_ACCELERATION = 8.5;
        public static final double MAX_ANGLE_VELOCITY = Math.PI;
        public static final double EXTENDED_MAX_ANGLE_VELOCITY_MULTIPLIER = 0.7;
        public static final double MAX_ANGLE_ACCELERATION = MAX_ANGLE_VELOCITY / 2.0;

    }

	public static final class Elevator 
    {
		public static final double MAX_ERROR = 1; // TODO rotations
												
		public static final double kP = 0.13; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO

		public static final double kG = 0.09; // TODO

        public static final InvertedValue MASTER_INVERTED = InvertedValue.Clockwise_Positive; // TODO
        public static final InvertedValue FOLLOWER_INVERTED = InvertedValue.Clockwise_Positive; // TODO

        public static final int MASTER_ID = 0; // TODO
        public static final int FOLLOWER_ID = 0; // TODO
        public static final int LIMIT_SWITCH_ID = 0; // TODO

        public static final double STATOR_CURRENT_LIMIT = 0; // TODO A
        public static final double FORWARD_SOFT_LIMIT = 0; // TODO rotations
        public static final double REVERSE_SOFT_LIMIT = 0; // TODO rotations

        public static final double FAR_EXTENDED_DISTANCE = 0; // TODO rotations

    }
    
    public static final class EndEffector
    {
        public static final int ID = 0; // TODO
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive; // TODO
        public static final int FORWARD_LIMIT_SWITCH_ID = 0; // TODO
        public static final int BACKWARD_LIMIT_SWITCH_ID = 0; // TODO
        public static final double ROT_TO_METERS = 0; // TODO

        public static final double kP = 0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO

        public static final double STATOR_CURRENT_LIMIT = 0; // TODO A
        public static final double FORWARD_SOFT_LIMIT = 0; // TODO rotations
        public static final double REVERSE_SOFT_LIMIT = 0; // TODO rotations

        public static final double ROLLER_SPEED = 0; // TODO meters / second
    }
}
