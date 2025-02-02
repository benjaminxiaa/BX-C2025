package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Flip;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance;

    private SwerveModule[] swerveModules;

    private Pigeon2 pigeon;
    private double prevHeading;

    // // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    // private final MutVoltage _appliedVoltage = mutable(Volts.of(0));
    // // Mutable holder for unit-safe linear distance values, persisted to avoid
    // // reallocation.
    // private final MutDistance _distance = mutable(Meters.of(0));
    // // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // // reallocation.
    // private final MutLinearVelocity _velocity = mutable(MetersPerSecond.of(0));

    private SwerveDriveKinematics kinematics; // converts chassis speeds (x, y, theta) to module states (speed, angle)

    // Estimates the robot's pose through encoder (state) and vision measurements;
    private SwerveDrivePoseEstimator poseEstimator;

    // Standard deviations of pose estimate (x, y, heading)
    private static Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.05); // increase to trust encoder (state)
                                                                                        // measurements less
    private static Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.5, 0.5, 0.5); // increase to trust vsion
                                                                                        // measurements less

    private boolean robotCentric;

    private Drivetrain() {
        // initialize swerve modules

        swerveModules = new SwerveModule[] {
                new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
        };

        // initialize pigeon
        pigeon = new Pigeon2(RobotMap.Drivetrain.PIGEON_ID, RobotMap.CAN_CHAIN);
        initPigeon();

        // initialize locations of swerve modules relative to robot (fl, fr, bl, br)
        kinematics = new SwerveDriveKinematics(
                new Translation2d(RobotMap.Drivetrain.ROBOT_LENGTH / 2, RobotMap.Drivetrain.ROBOT_WIDTH / 2),
                new Translation2d(RobotMap.Drivetrain.ROBOT_LENGTH / 2, -RobotMap.Drivetrain.ROBOT_WIDTH / 2),
                new Translation2d(-RobotMap.Drivetrain.ROBOT_LENGTH / 2, RobotMap.Drivetrain.ROBOT_WIDTH / 2),
                new Translation2d(-RobotMap.Drivetrain.ROBOT_LENGTH / 2, -RobotMap.Drivetrain.ROBOT_WIDTH / 2));


        // initial pose (holds the x, y, heading)
        Pose2d initalPoseMeters = new Pose2d();

        // initialize pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getRotation(),
                getModulePositions(),
                initalPoseMeters,
                stateStdDevs,
                visionStdDevs);
        
        robotCentric = false;
    }

    /*
     * Initialize pigeon values
     */
    private void initPigeon() {
        Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();
        pigeonConfigs.MountPose.MountPosePitch = 0;
        pigeonConfigs.MountPose.MountPoseRoll = 0;
        pigeonConfigs.MountPose.MountPoseYaw = 0;
        pigeonConfigs.Pigeon2Features.EnableCompass = false;
        pigeon.getYaw().setUpdateFrequency(250);

        pigeon.getConfigurator().apply(pigeonConfigs);
    }

    /**
     * Updates previous heading to the current heading or continues in the
     * same direction (omega becomes adjusted by the prevous heading)
     * 
     * @param omega rotational speed
     * @return adjusted rotational speed
     */
    public double adjustPigeon(double omega) {
        double currHeading = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        if (Math.abs(omega) <= RobotMap.Drivetrain.MIN_OUTPUT) {
            omega = -RobotMap.Drivetrain.PIGEON_kP * (prevHeading - currHeading);
        } else {
            prevHeading = currHeading;
        }

        return omega;
    }

    /**
     * @return pitch of pigeon in degrees
     */
    public double getPitch() {
        double pitch = pigeon.getPitch().getValueAsDouble();
        return pitch;
    }

    // public void setTranslationkP(double newkP) {
    // SmartDashboard.putNumber("newTranslationkP", newkP);
    // for(int i = 0; i<4;i++) {
    // swerveModules[i].setTranslationkP(newkP);
    // }
    // }

    // public void setTranslationkI(double newkI) {
    // SmartDashboard.putNumber("newTranslationkI", newkI);
    // for(int i = 0; i<4;i++) {
    // swerveModules[i].setTranslationkI(newkI);
    // }
    // }

    // public void setTranslationkD(double newkD) {
    // SmartDashboard.putNumber("newTranslationkD", newkD);
    // for(int i = 0; i<4;i++) {
    // swerveModules[i].setTranslationkD(newkD);
    // }
    // }

    // public void setRotationkP(double newkP) {
    // SmartDashboard.putNumber("newTranslationkP", newkP);
    // for(int i = 0; i<4;i++) {
    // swerveModules[i].setRotationkP(newkP);
    // }
    // }

    public boolean robotCentric() {
        return robotCentric;
    }

    public void toggleRobotCentric() {
        robotCentric = !robotCentric;
    }

    /**
     * @return heading of pigeon as a Rotation2d
     */
    public Rotation2d getRotation() {
        return pigeon.getRotation2d();
    }

    /**
     * @return the states of the swerve modules in an array
     */
    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                swerveModules[0].getSwerveModulePosition(),
                swerveModules[1].getSwerveModulePosition(),
                swerveModules[2].getSwerveModulePosition(),
                swerveModules[3].getSwerveModulePosition()
        };
    }

    /**
     * Sets the initial pose of the drivetrain
     * 
     * @param pose intial Pose2d of drivetrain
     */
    public void setPose(Pose2d pose) {
        pose = Flip.apply(pose);
        swerveModules[0].zeroTranslation();
        swerveModules[1].zeroTranslation();
        swerveModules[2].zeroTranslation();
        swerveModules[3].zeroTranslation();
        setYaw(pose.getRotation().getDegrees());
        poseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
    }

    /**
     * Sets the yaw of the pigeon to the given angle
     * 
     * @param yaw angle in degrees
     */
    public void setYaw(double yaw) {
        // System.out.println(yaw);
        pigeon.setYaw(yaw);
        setPreviousHeading(yaw);
    }

    /**
     * Updates previous heading
     * 
     * @param prev new heading
     */
    public void setPreviousHeading(double prev) {
        prevHeading = prev;
    }

    /**
     * @return kinematics of swerve drive
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Singleton code
     * 
     * @return instance of Drivetrain
     */
    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    /**
     * Converts chassis speeds to individual swerve module
     * states and sets the angle and drive for them
     * 
     * @param chassis chassis speeds to convert
     */
    public void setAngleAndDrive(ChassisSpeeds chassis) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassis);
        // SwerveDriveKinematics.desaturateWheelSpeeds(states,
        // RobotMap.SwerveModule.MAX_SPEED);
        swerveModules[0].setAngleAndDrive(states[0]);
        swerveModules[1].setAngleAndDrive(states[1]);
        swerveModules[2].setAngleAndDrive(states[2]);
        swerveModules[3].setAngleAndDrive(states[3]);
    }

    /**
     * Called every loop, feeds newest encoder readings to estimator
     */
    public void updatePose() {
        // System.out.println(getRotation() + " " + getModulePositions());
        poseEstimator.update(getRotation(), getModulePositions());
    }


    /**
     * @return the estimated pose as a Pose2d
     */
    public Pose2d getPoseEstimatorPose2d() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveModuleState[] getOptimizedStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.ID] = mod.getOptimizedModuleState();
        }

        return states;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.ID] = mod.getSwerveModuleState();
        }
        return states;
    }

    // private final SysIdRoutine _sysId = new SysIdRoutine(
    //         new SysIdRoutine.Config(),
    //         new SysIdRoutine.Mechanism(
    //                 (Voltage volts) -> {
    //                     runSwerveCharacterization(volts.in(Volts));
    //                 },
    //                 log -> {
    //                     log.motor("drive-left")
    //                             .voltage(
    //                                     _appliedVoltage.mut_replace(
    //                                             swerveModules[3].getTranslationVoltage(), Volts))
    //                             .linearPosition(
    //                                     _distance.mut_replace(swerveModules[3].getWheelPosition(), Meters))
    //                             .linearVelocity(
    //                                     _velocity.mut_replace(swerveModules[3].getSpeed(), MetersPerSecond));
    //                     log.motor("drive-right")
    //                             .voltage(
    //                                     _appliedVoltage.mut_replace(
    //                                             swerveModules[0].getTranslationVoltage(),
    //                                             Volts))
    //                             .linearPosition(
    //                                     _distance.mut_replace(swerveModules[3].getWheelPosition(), Meters))
    //                             .linearVelocity(
    //                                     _velocity.mut_replace(swerveModules[0].getSpeed(), MetersPerSecond));
    //                 },
    //                 this));

    public void runSwerveCharacterization(double volts) {
        for (SwerveModule module : swerveModules) {
            module.runCharacterization(volts);
        }
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return _sysId.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return _sysId.dynamic(direction);
    // }

    @Override
    public void periodic() {
        updatePose();

    }    
}
 