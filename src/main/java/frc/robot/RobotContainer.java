// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.EE.IntakeAlgae;
import frc.robot.commands.EE.Score;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.swerve.Modules;
import frc.robot.vision.VisionProcessor;
import harkerrobolib.joysticks.HSXboxController;

public class RobotContainer {
    private static RobotContainer instance = RobotContainer.getInstance();

    private double MaxSpeed = Constants.Swerve.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final VisionProcessor visionProcessor;

    private final HSXboxController driver = new HSXboxController(0);
    private final HSXboxController operator = new HSXboxController(1);

    public final Drivetrain drivetrain = Modules.createDrivetrain();
    private final Elevator elevator = Elevator.getInstance();
    private final EndEffector endEffector = EndEffector.getInstance();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("ZeroElevator", new ZeroElevator());
        NamedCommands.registerCommand("ElevatorHeight1",
                new MoveToPosition(Constants.Elevator.LEVEL_HEIGHTS[1]));
        NamedCommands.registerCommand("ElevatorHeight2",
                new MoveToPosition(Constants.Elevator.LEVEL_HEIGHTS[2]));
        NamedCommands.registerCommand("ElevatorHeight3",
                new MoveToPosition(Constants.Elevator.LEVEL_HEIGHTS[3]));
        NamedCommands.registerCommand("Score", new Score());

        autoChooser = AutoBuilder.buildAutoChooser("auton1");
        SmartDashboard.putData("Auton Chooser", autoChooser);

        visionProcessor = new VisionProcessor(
                drivetrain,
                Constants.Vision.LL4_CONFIG,
                Constants.Vision.LL3_CONFIG);

        SignalLogger.start();

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        elevator.setDefaultCommand(new ElevatorManual());

        endEffector.setDefaultCommand(
                new RunCommand(
                    () -> {
                        if (!endEffector.isBackTriggered() && !endEffector.isFrontTriggered())
                        {
                            endEffector.setSpeed((endEffector.isContinuousIntake()) ? Constants.EndEffector.INTAKE_CORAL_SLOW_SPEED : 0);
                        }
                        else if (endEffector.isBackTriggered() && !endEffector.isFrontTriggered())
                        {
                            endEffector.setSpeed(Constants.EndEffector.INTAKE_CORAL_SPEED);
                        }
                        else if (!EndEffector.getInstance().isBackTriggered() && EndEffector.getInstance().isFrontTriggered())
                        {
                            endEffector.setSpeed(Constants.EndEffector.EJECT_SPEED);
                        }
                        else
                        {
                            endEffector.setSpeed(0);
                        }
                    },
                    endEffector));

        // reset the field-centric heading on button b press
        driver.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driver.rightBumper().onTrue(new Score()
                .andThen(new MoveToPosition(0)
                        .andThen(new ZeroElevator())));

        driver.x().onTrue(new Score());

        driver.leftBumper().onTrue(new IntakeAlgae());

        driver.y().whileTrue(new ZeroElevator());

        operator.x().onTrue(new MoveToPosition(0).andThen(new ZeroElevator()));
        operator.y().onTrue(new MoveToPosition(Constants.Elevator.LEVEL_HEIGHTS[3]));
        operator.b().onTrue(new MoveToPosition(Constants.Elevator.LEVEL_HEIGHTS[2]));
        operator.a().onTrue(new MoveToPosition(Constants.Elevator.LEVEL_HEIGHTS[1]));

        operator.leftBumper().onTrue(new MoveToPosition(Constants.Elevator.ALGAE_HEIGHTS[0]));
        operator.rightBumper().onTrue(new MoveToPosition(Constants.Elevator.ALGAE_HEIGHTS[1]));

        operator.getLeftDPad().onTrue(endEffector.runOnce(() -> endEffector.toggleContinousIntake()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterizeDrive);
        logger.telemeterize(elevator, endEffector);
        updateTelemetry();
    }

    public void updateTelemetry() {
        logger.telemeterize(elevator, endEffector);
    }

    public void updateVision() {
        visionProcessor.update();

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public HSXboxController getOperator() {
        return operator;
    }

    public static RobotContainer getInstance() {
        if (instance == null)
            instance = new RobotContainer();
        return instance;
    }
}