package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotMap;

public class Flip {
    /**
     * Flips the Translation2d based on current alliance
     */
    public static Translation2d apply(Translation2d translation2d) {
        return (isFlipped())
                ? new Translation2d(RobotMap.Field.FIELD_LENGTH - translation2d.getX(),
                        translation2d.getY())
                : translation2d;
    }

    /**
     * Flips the Rotation2d based on current alliance
     */

    public static Rotation2d apply(Rotation2d rotation2d) {
        return (isFlipped()) ? new Rotation2d(-rotation2d.getCos(), rotation2d.getSin()) : rotation2d;
    }

    /**
     * Flips the Pose2d based on current alliance
     */

    public static Pose2d apply(Pose2d pose2d) {
        return (isFlipped())
                ? new Pose2d(RobotMap.Field.FIELD_LENGTH - pose2d.getX(), pose2d.getY(),
                        new Rotation2d(-pose2d.getRotation().getCos(), pose2d.getRotation().getSin()))
                : pose2d;
    }

    /**
     * Flips the Trajectory.State based on current alliance
     */

    public static Trajectory.State apply(Trajectory.State state) {
        return (isFlipped()) ? new Trajectory.State(
                state.timeSeconds,
                state.velocityMetersPerSecond,
                state.accelerationMetersPerSecondSq,
                new Pose2d(
                        RobotMap.Field.FIELD_LENGTH - state.poseMeters.getX(),
                        state.poseMeters.getY(),
                        new Rotation2d(
                                -state.poseMeters.getRotation().getCos(),
                                state.poseMeters.getRotation().getSin())),
                -state.curvatureRadPerMeter) : state;
    }

    /**
     * if Alliance is red, flip the trajectory
     * field flipped along y-axis
     */
    public static boolean isFlipped() {
        if (DriverStation.getAlliance().isPresent())
            return DriverStation.getAlliance().get() == Alliance.Red;
        return false;
    }
}
