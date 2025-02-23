// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.drivetrain.AutoAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.util.SwerveGenerator;
import frc.robot.vision.VisionProcessor;

public class RobotContainer {
    private double MaxSpeed = Constants.Swerve.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final VisionProcessor visionProcessor;

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final Drivetrain drivetrain = SwerveGenerator.createDrivetrain();
    private final Elevator elevator = Elevator.getInstance();

    /* Path follower */
    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // autoChooser = AutoBuilder.buildAutoChooser("Tests");
        // SmartDashboard.putData("Auto Mode", autoChooser);
        // SignalLogger.setPath("/media/sda1/");

        visionProcessor = new VisionProcessor(
                drivetrain,
                Constants.Vision.LL4_CONFIG,
                Constants.Vision.LL3_CONFIG);

        SignalLogger.start();
        DataLogManager.start();

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        elevator.setDefaultCommand(
            new RunCommand(
                () -> {
                    // Get elevator movement from D-pad
                    double movement = 0;
                    if (operator.povUp().getAsBoolean()) movement = 0.2;
                    else if (operator.povDown().getAsBoolean()) movement = -0.2;
                    
                    if (movement != 0) {
                        elevator.setDesiredPosition(elevator.getPosition() + movement);
                        elevator.setManual(true);
                    } else if (elevator.isManual()) {
                        elevator.setDesiredPosition(elevator.getPosition());
                    }
                    
                    elevator.moveToPosition();
                },
                elevator
            )
        );

        driver.rightBumper().whileTrue(new AutoAlign(drivetrain));

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        driver.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        driver.pov(180)
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void updateVision() {
        visionProcessor.update();
    }

    // public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    // return autoChooser.getSelected();
    // }
}