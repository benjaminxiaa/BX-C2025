package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

public class VisionProcessor {
    private final Drivetrain drivetrain;

    // Configuration for each Limelight
    public static class LimelightConfig {
        public final String name;
        public final Transform3d robotToCamera;
        public final double minTagArea;
        public final double minTagSpan;
        public final int minTagCount;
        public final boolean useMegaTag2;

        public LimelightConfig(String name, Transform3d robotToCamera,
                double minTagArea, double minTagSpan,
                int minTagCount, boolean useMegaTag2) {
            this.name = name;
            this.robotToCamera = robotToCamera;
            this.minTagArea = minTagArea;
            this.minTagSpan = minTagSpan;
            this.minTagCount = minTagCount;
            this.useMegaTag2 = useMegaTag2;
        }
    }

    private static final double MAX_POSE_CHANGE_METERS = 0.5;
    private static final double MAX_ANGULAR_VELOCITY = Units.rotationsToRadians(2.0);

    // Configurations for each Limelight
    private final LimelightConfig[] limelightConfigs;

    // Tracking last poses for each Limelight
    private final Pose2d[] lastPoses;
    private final double[] lastTimestamps;

    public VisionProcessor(Drivetrain drivetrain, LimelightConfig... configs) {
        this.drivetrain = drivetrain;
        this.limelightConfigs = configs;
        this.lastPoses = new Pose2d[configs.length];
        this.lastTimestamps = new double[configs.length];

        for (int i = 0; i < configs.length; i++) {
            lastPoses[i] = new Pose2d();
            lastTimestamps[i] = 0;
        }
    }

    public void update() {
        var driveState = drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRad = driveState.Speeds.omegaRadiansPerSecond;

        // Process each Limelight
        for (int i = 0; i < limelightConfigs.length; i++) {
            LimelightConfig config = limelightConfigs[i];

            // Set robot orientation for MegaTag 2 if enabled
            if (config.useMegaTag2) {
                LimelightHelpers.SetRobotOrientation_NoFlush(config.name, headingDeg, 0, 0, 0, 0, 0);
            }

            // Get pose estimate
            var llMeasurement = config.useMegaTag2 ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.name)
                    : LimelightHelpers.getBotPoseEstimate_wpiBlue(config.name);

            if (llMeasurement != null && isValidMeasurement(llMeasurement, omegaRad, config, i)) {
                // Calculate standard deviations based on measurement quality
                Matrix<N3, N1> stdDevs = calculateStandardDeviation(llMeasurement, config);

                // Add vision measurement with calculated confidence
                drivetrain.addVisionMeasurement(
                        llMeasurement.pose,
                        llMeasurement.timestampSeconds,
                        stdDevs);

                lastPoses[i] = llMeasurement.pose;
                lastTimestamps[i] = llMeasurement.timestampSeconds;
            }
        }
    }

    private boolean isValidMeasurement(LimelightHelpers.PoseEstimate measurement,
            double omegaRad,
            LimelightConfig config,
            int configIndex) {
        // Check tag count threshold
        if (measurement.tagCount < config.minTagCount)
            return false;

        // Reject if robot is rotating too fast
        if (Math.abs(omegaRad) > MAX_ANGULAR_VELOCITY)
            return false;

        // Check tag area
        if (measurement.avgTagArea < config.minTagArea)
            return false;

        // Check for unreasonable pose jumps
        if (lastTimestamps[configIndex] != 0) {
            double poseChange = measurement.pose.getTranslation()
                    .getDistance(lastPoses[configIndex].getTranslation());
            if (poseChange > MAX_POSE_CHANGE_METERS)
                return false;
        }

        // Verify tag span
        if (measurement.tagSpan < config.minTagSpan)
            return false;

        return true;
    }

    private Matrix<N3, N1> calculateStandardDeviation(LimelightHelpers.PoseEstimate measurement,
            LimelightConfig config) {
        // Base standard deviation - higher means less confidence
        double baseStdDev = 1.0;

        // Reduce standard deviation (increase confidence) with more tags
        baseStdDev /= Math.sqrt(measurement.tagCount);

        // Reduce confidence based on distance
        baseStdDev *= (1.0 + measurement.avgTagDist * 0.2);

        // Increase confidence with larger tag span
        baseStdDev /= (1.0 + measurement.tagSpan * 0.1);

        // Create the standard deviation vector - typically want higher rotational
        // uncertainty
        return VecBuilder.fill(baseStdDev, baseStdDev, baseStdDev * 2);
    }
}