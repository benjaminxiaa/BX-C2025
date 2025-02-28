package frc.robot.simulation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.LimelightHelpers;

import java.util.ArrayList;
import java.util.List;

public class LimelightSimulation {
    // The network table used by the simulated Limelight
    private final NetworkTable limelightTable;

    // The simulated camera position relative to robot center
    private final Transform3d robotToCamera;

    // Field visualization
    private final Field2d field2d;

    // List of simulated AprilTags on the field
    private final List<SimulatedAprilTag> aprilTags = new ArrayList<>();

    // Maximum distance at which tags can be detected
    private static final double MAX_VISION_RANGE_METERS = 6.0;

    // Field of view limits (in degrees)
    private static final double HORIZONTAL_FOV_DEG = 60.0;
    private static final double VERTICAL_FOV_DEG = 45.0;

    public LimelightSimulation(String limelightName, Transform3d robotToCamera) {
        this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
        this.robotToCamera = robotToCamera;
        this.field2d = new Field2d();

        // Initialize with 2024 CRESCENDO field AprilTag positions
        for (var tagPose : AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo).getTags()) {
            aprilTags.add(new SimulatedAprilTag(tagPose.ID, tagPose.pose));
        }
    }

    /**
     * Update the simulated Limelight based on current robot pose
     * 
     * @param robotPose Current robot pose in field coordinates
     */
    public void update(Pose2d robotPose) {
        // Convert 2d robot pose to 3d
        Pose3d robotPose3d = new Pose3d(
                robotPose.getX(),
                robotPose.getY(),
                0.0,
                new Rotation3d(0, 0, robotPose.getRotation().getRadians()));

        // Get camera pose in field coordinates
        Pose3d cameraPose = robotPose3d.transformBy(robotToCamera);

        List<SimulatedAprilTag> visibleTags = findVisibleTags(cameraPose);

        // Update NetworkTables with simulated values
        limelightTable.getEntry("tv").setDouble(visibleTags.isEmpty() ? 0.0 : 1.0);

        if (!visibleTags.isEmpty()) {
            // Use the closest visible tag for pose estimation
            SimulatedAprilTag closestTag = visibleTags.get(0);

            // Calculate target information relative to camera
            Transform3d cameraToTarget = cameraPose.minus(closestTag.pose);

            // Update basic targeting data
            double tx = Units.radiansToDegrees(Math.atan2(cameraToTarget.getY(), cameraToTarget.getX()));
            double ty = Units.radiansToDegrees(Math.atan2(cameraToTarget.getZ(), cameraToTarget.getX()));

            limelightTable.getEntry("tx").setDouble(tx);
            limelightTable.getEntry("ty").setDouble(ty);
            limelightTable.getEntry("ta").setDouble(calculateTargetArea(cameraToTarget));
            limelightTable.getEntry("tid").setDouble(closestTag.id);

            // Update pose estimation
            double[] botpose = LimelightHelpers.pose2dToArray(robotPose);
            limelightTable.getEntry("botpose").setDoubleArray(botpose);
            limelightTable.getEntry("botpose_wpiblue").setDoubleArray(botpose);

            // Calculate target space transforms
            Transform3d targetToCamera = closestTag.pose.minus(cameraPose);
            limelightTable.getEntry("targetpose_cameraspace").setDoubleArray(
                    LimelightHelpers.pose3dToArray(new Pose3d().plus(targetToCamera)));
        } else {
            // No targets visible - clear values
            limelightTable.getEntry("tx").setDouble(0.0);
            limelightTable.getEntry("ty").setDouble(0.0);
            limelightTable.getEntry("ta").setDouble(0.0);
            limelightTable.getEntry("tid").setDouble(-1);
        }

        // Update field visualization
        field2d.setRobotPose(robotPose);
    }

    private List<SimulatedAprilTag> findVisibleTags(Pose3d cameraPose) {
        List<SimulatedAprilTag> visibleTags = new ArrayList<>();

        for (SimulatedAprilTag tag : aprilTags) {
            Transform3d cameraToTarget = cameraPose.minus(tag.pose);

            // Check if tag is within range
            double distance = cameraToTarget.getTranslation().getNorm();
            if (distance > MAX_VISION_RANGE_METERS) {
                continue;
            }

            // Check if tag is within FOV
            double horizontalAngle = Math.abs(Units.radiansToDegrees(
                    Math.atan2(cameraToTarget.getY(), cameraToTarget.getX())));
            double verticalAngle = Math.abs(Units.radiansToDegrees(
                    Math.atan2(cameraToTarget.getZ(), cameraToTarget.getX())));

            if (horizontalAngle <= HORIZONTAL_FOV_DEG / 2 && verticalAngle <= VERTICAL_FOV_DEG / 2) {
                visibleTags.add(tag);
            }
        }

        // Sort by distance
        visibleTags.sort((a, b) -> {
            double distA = cameraPose.minus(a.pose).getTranslation().getNorm();
            double distB = cameraPose.minus(b.pose).getTranslation().getNorm();
            return Double.compare(distA, distB);
        });

        return visibleTags;
    }

    private double calculateTargetArea(Transform3d cameraToTarget) {
        // Simulate target area based on distance
        double distance = cameraToTarget.getTranslation().getNorm();
        // This is a simplified model - you may want to tune these values
        return Math.min(100.0, 200.0 / (distance * distance));
    }

    public Field2d getField2d() {
        // Update AprilTag poses on field visualization
        for (SimulatedAprilTag tag : aprilTags) {
            field2d.getObject("AprilTag " + tag.id)
                    .setPose(new Pose2d(
                            tag.pose.getX(),
                            tag.pose.getY(),
                            new Rotation2d(tag.pose.getRotation().getZ())));
        }
        return field2d;
    }

    /**
     * Gets the recommended alignment pose for a given AprilTag ID
     * 
     * @param tagId    The ID of the AprilTag to align with
     * @param distance Desired distance from the tag in meters
     * @return The recommended robot pose for alignment, or null if tag ID is
     *         invalid
     */
    public Pose2d getAlignmentPose(int tagId, double distance) {
        for (SimulatedAprilTag tag : aprilTags) {
            if (tag.id == tagId) {
                // For speaker tags (1-3 blue, 6-8 red)
                if ((tagId >= 1 && tagId <= 3) || (tagId >= 6 && tagId <= 8)) {
                    // Face directly towards the speaker
                    double angleToTag = tag.pose.getRotation().getZ();
                    return new Pose2d(
                            tag.pose.getX() - distance * Math.cos(angleToTag),
                            tag.pose.getY() - distance * Math.sin(angleToTag),
                            new Rotation2d(angleToTag));
                }
                // For amp tags (12-14)
                else if (tagId >= 12 && tagId <= 14) {
                    // Align parallel to the amp
                    double angleToTag = tag.pose.getRotation().getZ();
                    return new Pose2d(
                            tag.pose.getX() - distance * Math.cos(angleToTag),
                            tag.pose.getY() - distance * Math.sin(angleToTag),
                            new Rotation2d(angleToTag + Math.PI / 2) // 90 degrees to face amp
                    );
                }
                // For stage tags (9-11, 15-16)
                else {
                    // Default to facing the tag directly
                    double angleToTag = tag.pose.getRotation().getZ();
                    return new Pose2d(
                            tag.pose.getX() - distance * Math.cos(angleToTag),
                            tag.pose.getY() - distance * Math.sin(angleToTag),
                            new Rotation2d(angleToTag));
                }
            }
        }
        return null;
    }

    private static class SimulatedAprilTag {
        public final int id;
        public final Pose3d pose;

        public SimulatedAprilTag(int id, Pose3d pose) {
            this.id = id;
            this.pose = pose;
        }
    }
}