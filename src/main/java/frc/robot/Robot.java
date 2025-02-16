// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.EE.IntakeCoral;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.Telemetry;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   private Telemetry telemetry;
  @Override
  public void robotInit() {
      CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new SwerveManual());
      CommandScheduler.getInstance().setDefaultCommand(Elevator.getInstance(), new ElevatorManual());
      CommandScheduler.getInstance().setDefaultCommand(EndEffector.getInstance(), new IntakeCoral());
      
      telemetry = new Telemetry();
      // telemetry.startServer();
      telemetry.swerveStates();
      // CanandEventLoop.getInstance();
    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // RobotMap.Field.FIELD.setRobotPose(new Pose2d(0,0,new Rotation2d(0)));
    RobotMap.Field.FIELD.setRobotPose(Drivetrain.getInstance().getPoseEstimatorPose2d());
    telemetry.publish();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
