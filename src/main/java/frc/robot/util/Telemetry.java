package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.OI;
import frc.robot.subsystems.swerve.Drivetrain;

public class Telemetry {
    // private NetworkTable table;
    private NetworkTableInstance inst;

    private static NetworkTable main;

    // private static NetworkTable _realOutputs;
    private static NetworkTable _swerve;

    private NetworkTable _odometry;
    private NetworkTable _autons;

    private static NetworkTable _drive;
    private static NetworkTable _elevator;
    private static NetworkTable _pivot;
    private static NetworkTable _shooter;
    private static NetworkTable _intake;

    private NetworkTable _modules;
    private static NetworkTable _zero;
    private static NetworkTable _one;
    private static NetworkTable _two;
    private static NetworkTable _three;

    private NetworkTable _vision;
    private NetworkTable _targets;
    private NetworkTable _limelight;

    private NetworkTable _debug;

    private NetworkTable _controls;
    private NetworkTable _driver;
    private NetworkTable _operator;

    private Drivetrain drive = Drivetrain.getInstance();

    private XboxGamepad oiDriver = OI.getInstance().getDriver();

    private StructArrayPublisher<SwerveModuleState> swerveModuleStates;
    private StructArrayPublisher<SwerveModuleState> swerveModuleStpsOptmized;

    public Telemetry() {
        // main = NetworkTableInstance.getDefault();
        inst = NetworkTableInstance.getDefault();

        main = inst.getTable("TEAM 1072");

        _swerve = main.getSubTable("Swerve");

        // _realOutputs = main.getSubTable("Real Outputs");

        _odometry = main.getSubTable("Odometry");
        _autons = main.getSubTable("Autons");

        _drive = main.getSubTable("Drive");

        _modules = _drive.getSubTable("Modules");
        _zero = _modules.getSubTable("0");
        _one = _modules.getSubTable("1");
        _two = _modules.getSubTable("2");
        _three = _modules.getSubTable("3");

        _elevator = main.getSubTable("Elevator");
        _pivot = main.getSubTable("Pivot");
        _shooter = main.getSubTable("Shooter");
        _intake = main.getSubTable("Intake");

        _vision = main.getSubTable("Vision");
        _targets = _vision.getSubTable("At Targets");
        _limelight = _vision.getSubTable("Limelight");

        _debug = main.getSubTable("Debug");
        _controls = _debug.getSubTable("Controls");
        _driver = _controls.getSubTable("Driver");
        _operator = _controls.getSubTable("Operator");
    }

    public void autons(String entry, String list) {
        NetworkTableEntry cAuton = _autons.getEntry(entry);
        cAuton.setString(list);
    }

    public void odometry() {
        NetworkTableEntry rotation = _odometry.getEntry("Rotation");
        rotation.setDouble(drive.getRotation().getDegrees());


        NetworkTableEntry poseRotation = _odometry.getEntry("Pose Rotation in Deg");
        poseRotation.setDouble(drive.getPoseEstimatorPose2d().getRotation().getDegrees());
    }

    public void debug() {
        // NetworkTableEntry a_D = _driver.getEntry("Button A");
        // a_D.setBoolean(oiDriver.getButtonAState());

        // NetworkTableEntry b_D = _driver.getEntry("Button B");
        // b_D.setBoolean(oiDriver.getButtonBState());

        // NetworkTableEntry x_D = _driver.getEntry("Button X");
        // x_D.setBoolean(oiDriver.getButtonXState());

        // NetworkTableEntry y_D = _driver.getEntry("Button Y");
        // y_D.setBoolean(oiDriver.getButtonYState());

        // NetworkTableEntry leftBumper_D = _driver.getEntry("Left Bumper");
        // leftBumper_D.setBoolean(oiDriver.getLeftBumperState());

        // NetworkTableEntry rightBumper_D = _driver.getEntry("Right Bumper");
        // rightBumper_D.setBoolean(oiDriver.getRightBumperState());

        // NetworkTableEntry leftTrigger_D = _driver.getEntry("Left Trigger");
        // leftTrigger_D.setDouble(oiDriver.getLeftTrigger());

        // NetworkTableEntry rightTrigger_D = _driver.getEntry("Right Trigger");
        // rightTrigger_D.setDouble(oiDriver.getRightTrigger());

        // NetworkTableEntry upD_D = _driver.getEntry("Up DPad");
        // upD_D.setBoolean(oiDriver.getUpDPadButtonState());

        // NetworkTableEntry leftD_D = _driver.getEntry("Left DPad");
        // leftD_D.setBoolean(oiDriver.getLeftDPadButtonState());

        // NetworkTableEntry downD_D = _driver.getEntry("Down DPad");
        // downD_D.setBoolean(oiDriver.getDownDPadButtonState());

        // NetworkTableEntry rightD_D = _driver.getEntry("Right DPad");
        // rightD_D.setBoolean(oiDriver.getDownDPadButtonState());

        // NetworkTableEntry select_D = _driver.getEntry("Select");
        // select_D.setBoolean(oiDriver.getButtonSelectState());

        // NetworkTableEntry start_D = _driver.getEntry("Start");
        // start_D.setBoolean(oiDriver.getButtonStartState());

        // NetworkTableEntry a = _operator.getEntry("Button A");
        // a.setBoolean(oiOperator.getButtonAState());

        // NetworkTableEntry b = _operator.getEntry("Button B");
        // b.setBoolean(oiOperator.getButtonBState());

        // NetworkTableEntry x = _operator.getEntry("Button X");
        // x.setBoolean(oiOperator.getButtonXState());

        // NetworkTableEntry y = _operator.getEntry("Button Y");
        // y.setBoolean(oiOperator.getButtonYState());

        // NetworkTableEntry leftBumper = _operator.getEntry("Left Bumper");
        // leftBumper.setBoolean(oiOperator.getLeftBumperState());

        // NetworkTableEntry rightBumper = _operator.getEntry("Right Bumper");
        // rightBumper.setBoolean(oiOperator.getRightBumperState());

        // NetworkTableEntry leftTrigger = _operator.getEntry("Left Trigger");
        // leftTrigger.setDouble(oiOperator.getLeftTrigger());

        // NetworkTableEntry rightTrigger = _operator.getEntry("Right Trigger");
        // rightTrigger.setDouble(oiOperator.getRightTrigger());

        // NetworkTableEntry upD = _operator.getEntry("Up DPad");
        // upD.setBoolean(oiOperator.getUpDPadButtonState());

        // NetworkTableEntry leftD = _operator.getEntry("Left DPad");
        // leftD.setBoolean(oiOperator.getLeftDPadButtonState());

        // NetworkTableEntry downD = _operator.getEntry("Down DPad");
        // downD.setBoolean(oiOperator.getDownDPadButtonState());

        // NetworkTableEntry rightD = _operator.getEntry("Right DPad");
        // rightD.setBoolean(oiOperator.getDownDPadButtonState());

        // NetworkTableEntry select = _operator.getEntry("Select");
        // select.setBoolean(oiOperator.getButtonSelectState());

        // NetworkTableEntry start = _operator.getEntry("Start");
        // start.setBoolean(oiOperator.getButtonStartState());

        // NetworkTableEntry isRobotCentric = _debug.getEntry("Robot Centric");
        // isRobotCentric.setBoolean(drive.robotCentric());
    }


    public static void putModule(int id, String entry, double number) {
        switch (id) {
            case 0:
                NetworkTableEntry zeroName = _zero.getEntry("Module Name");
                zeroName.setString("Top Left");

                NetworkTableEntry _entry0 = _zero.getEntry(entry);
                _entry0.setDouble(number);
                break;

            case 1:
                NetworkTableEntry oneName = _one.getEntry("Module Name");
                oneName.setString("Top Right");

                NetworkTableEntry _entry1 = _one.getEntry(entry);
                _entry1.setDouble(number);
                break;

            case 2:
                NetworkTableEntry twoName = _two.getEntry("Module Name");
                twoName.setString("Bottom Left");

                NetworkTableEntry _entry2 = _two.getEntry(entry);
                _entry2.setDouble(number);
                break;

            case 3:
                NetworkTableEntry threeName = _three.getEntry("Module Name");
                threeName.setString("Bottom Right");

                NetworkTableEntry _entry3 = _three.getEntry(entry);
                _entry3.setDouble(number);
                break;
        }
    }

    public static void putNumber(String system, String entry, double number) {
        switch (system) {
            case "swerve":
                NetworkTableEntry _entrySwerve = _swerve.getEntry(entry);
                _entrySwerve.setDouble(number);
                break;

            default:
                break;
        }
    }

    public static void putBoolean(String system, String entry, boolean bool) {
        switch (system) {
            case "swerve":
                NetworkTableEntry _entrySwerve = _swerve.getEntry(entry);
                _entrySwerve.setBoolean(bool);
                break;

            default:
                break;
        }
    }

    public static void putString(String system, String entry, String string) {
        switch (system) {
            case "swerve":
                NetworkTableEntry _entrySwerve = _swerve.getEntry(entry);
                _entrySwerve.setString(string);
                break;

            default:
                break;
        }
    }

    public void swerveStates() {
        swerveModuleStates = _drive.getSubTable("SwerveModuleStates")
                .getStructArrayTopic("Measured", SwerveModuleState.struct).publish();

        swerveModuleStpsOptmized = _drive.getSubTable("SwerveModuleStates")
                .getStructArrayTopic("Setpoints Optimized", SwerveModuleState.struct).publish();
    }

    public void publish() {
        swerveModuleStates.set(drive.getModuleStates());
        swerveModuleStpsOptmized.set(drive.getOptimizedStates());

        debug();
        odometry();

        inst.flushLocal();
        inst.flush();
    }
}