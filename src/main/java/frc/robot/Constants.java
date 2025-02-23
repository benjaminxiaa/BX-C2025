package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.vision.VisionProcessor;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

public class Constants {
    public static class Vision {
        public static final String kCamera1Name = "limelight4"; // left limelight?
        public static final Transform3d kRobotToCam1 = new Transform3d(
                new Translation3d(10.834, -4.82, 9.065598), // TODO
                new Rotation3d(0, 0, 11.642)); // TODO

        public static final String kCamera2Name = "limelight3"; // right limelight
        public static final Transform3d kRobotToCam2 = new Transform3d(
                new Translation3d(12.905, 1.715, 16.065598), // TODO
                new Rotation3d(0, 0, 0)
        );

        // Vision processing parameters
        public static final VisionProcessor.LimelightConfig LL4_CONFIG = new VisionProcessor.LimelightConfig(
                kCamera1Name,
                kRobotToCam1,
                0.15, // minTagArea
                1.0, // minTagSpan
                2, // minTagCount
                true // use MegaTag2
        );

        public static final VisionProcessor.LimelightConfig LL3_CONFIG = new VisionProcessor.LimelightConfig(
                kCamera2Name,
                kRobotToCam2,
                0.15, // minTagArea
                1.0, // minTagSpan
                2, // minTagCount
                false // not using MegaTag2
        );

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class Sim {
        public static final double kSimLoopPeriod = 0.02; // 5 ms
    }

    public static class Drive {
        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        public static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        public static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

        public static final double MAX_TRANSLATION_SPEED = 1.0; // m/s
        public static final double MAX_ROTATION_SPEED = 1.0; // rad/s

        public static final double xAlignKP = 0.03; // TODO
        public static final double xAlignKI = 0; // TODO
        public static final double xAlignKD = 0; // TODO
        
        public static final double rotAlignKP = 0.03; // TODO
        public static final double rotAlignKI = 0; // TODO
        public static final double rotAlignKD = 0; // TODO
    }

    public static class Swerve {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        public static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.5)
                .withKS(0.1).withKV(2.66).withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        public static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(0.1).withKI(0).withKD(0)
                .withKS(0).withKV(0.124);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        public static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        public static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
        public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        public static final Current kSlipCurrent = Amps.of(120.0);

        // Initial configs for the drive and steer motors and the azimuth encoder; these
        // cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API
        // documentation.
        public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                // Swerve azimuth does not require much torque output, so we can set a
                                // relatively low
                                // stator current limit to help avoid brownouts without impacting performance.
                                .withStatorCurrentLimit(Amps.of(60))
                                .withStatorCurrentLimitEnable(true));
        public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        public static final Pigeon2Configuration pigeonConfigs = null;

        // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus kCANBus = new CANBus("rio");

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(9.53);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        public static final double kCoupleRatio = 0;

        public static final double kDriveGearRatio = 6.696;
        public static final double kSteerGearRatio = 21.42;
        public static final Distance kWheelRadius = Inches.of(4);

        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        public static final int kPigeonId = 1;

        // These are only used for simulation
        public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        // Front Left
        public static final int kFrontLeftDriveMotorId = 3;
        public static final int kFrontLeftSteerMotorId = 2;
        public static final int kFrontLeftEncoderId = 1;
        public static final Angle kFrontLeftEncoderOffset = Rotations.of(0.031982421875);
        public static final boolean kFrontLeftSteerMotorInverted = false;
        public static final boolean kFrontLeftEncoderInverted = true;

        public static final Distance kFrontLeftXPos = Inches.of(14);
        public static final Distance kFrontLeftYPos = Inches.of(15);

        // Front Right
        public static final int kFrontRightDriveMotorId = 1;
        public static final int kFrontRightSteerMotorId = 0;
        public static final int kFrontRightEncoderId = 0;
        public static final Angle kFrontRightEncoderOffset = Rotations.of(0.167236328125);
        public static final boolean kFrontRightSteerMotorInverted = false;
        public static final boolean kFrontRightEncoderInverted = true;

        public static final Distance kFrontRightXPos = Inches.of(14);
        public static final Distance kFrontRightYPos = Inches.of(-15);

        // Back Left
        public static final int kBackLeftDriveMotorId = 7;
        public static final int kBackLeftSteerMotorId = 6;
        public static final int kBackLeftEncoderId = 3;
        public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.116943359375);
        public static final boolean kBackLeftSteerMotorInverted = false;
        public static final boolean kBackLeftEncoderInverted = true;

        public static final Distance kBackLeftXPos = Inches.of(-14);
        public static final Distance kBackLeftYPos = Inches.of(15);

        // Back Right
        public static final int kBackRightDriveMotorId = 5;
        public static final int kBackRightSteerMotorId = 4;
        public static final int kBackRightEncoderId = 2;
        public static final Angle kBackRightEncoderOffset = Rotations.of(0.013427734375);
        public static final boolean kBackRightSteerMotorInverted = false;
        public static final boolean kBackRightEncoderInverted = true;

        public static final Distance kBackRightXPos = Inches.of(-14);
        public static final Distance kBackRightYPos = Inches.of(-15);
    }
}
