package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;
import frc.robot.lib.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.*;
import java.util.concurrent.*;

/**
 * Handles all camera calculations on the robot.
 *
 * <p>This is primarily used for combining MegaTag pose estimates and ensuring no conflicts between
 * Limelights.
 *
 * <p>Our vision system consists of: - 1 Limelight 4 (center back of the elevator crossbeam) - 1
 * Limelight 4 (under the pivot, 20-degree inclination) - 2 Limelight 3As (front swerve covers,
 * 15-degree outward incline)
 *
 * <p>We use the starting position in auto to determine our robot heading to calibrate our cameras.
 */
public class VisionSubsystem extends StateSubsystem<VisionSubsystem.SubsystemState> {

  /** Defines what vision data sources are used for pose estimation. */
  public enum SubsystemState {
    /** Enables MegaTag 2 pose estimates from all tags. */
    ALL_ESTIMATES(
        Arrays.asList(
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22)),

    /** Only allows pose estimates from reef-related AprilTags. */
    REEF_ESTIMATES(Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22)),

    /** Ignores all Limelight pose estimates. */
    NO_ESTIMATES(Collections.emptyList());

    private final List<Integer> tagIds;

    SubsystemState(List<Integer> tagIds) {
      this.tagIds = tagIds;
    }

    public List<Integer> getTagIds() {
      return tagIds;
    }
  }

  private final SwerveSubsystem swerve;
  private final List<String> cameras;
  private final ExecutorService executor;

  // Publishing for debugging/logging
  private final StructArrayPublisher<Pose3d> visibleTagsPub =
      getNetworkTable().getStructArrayTopic("Visible Tags", Pose3d.struct).publish();
  private final StructPublisher<Pose2d> finalMeasurementPub =
      getNetworkTable().getStructTopic("Best Measurement", Pose2d.struct).publish();
  private final StructArrayPublisher<Pose2d> visionMeasurementsPub =
      getNetworkTable().getStructArrayTopic("All Measurements", Pose2d.struct).publish();

  /**
   * Constructs the VisionSubsystem.
   *
   * @param swerve The swerve subsystem reference
   * @param cameras The list of Limelight camera names
   */
  public VisionSubsystem(SwerveSubsystem swerve, String... cameras) {
    super("Vision", SubsystemState.ALL_ESTIMATES);
    this.swerve = swerve;
    this.cameras = Arrays.asList(cameras);

    for (String cam : this.cameras) {
      if (cam == null || cam.isEmpty()) {
        throw new IllegalArgumentException("All camera names must be valid strings.");
      }
    }

    // Creates threads for each camera respectively
    this.executor = Executors.newFixedThreadPool(this.cameras.size());
  }

  @Override
  public void periodic() {
    super.periodic();

    // Don't read cameras if we've set it to NO_ESTIMATES or if we're spinning quickly (too much blur, saves computing)
    SubsystemState state = getCurrentState();
    if (state == SubsystemState.NO_ESTIMATES
        || Math.abs(swerve.getPigeon2().getAngularVelocityZWorld().getValue().in(DegreesPerSecond))
            > 720) {
      return;
    }

    // Process each camera in parallel
    List<Future<CameraResult>> futures =
        cameras.stream().map(cam -> executor.submit(() -> processCamera(cam))).toList();

    LimelightHelpers.PoseEstimate bestEstimate = null;
    List<Pose3d> visibleTags = new ArrayList<>();
    List<Pose2d> allMeasurements = new ArrayList<>();

    for (Future<CameraResult> future : futures) {
      try {
        CameraResult result = future.get();
        LimelightHelpers.PoseEstimate estimate = result.estimate;

        // Ignore bad estimates
        if (estimate == null || estimate.tagCount == 0) continue;

        // Add all tags this estimate sees
        visibleTags.addAll(
            Arrays.stream(estimate.rawFiducials)
                .map(fid -> Constants.FIELD_LAYOUT.getTagPose(fid.id))
                .flatMap(Optional::stream)
                .toList());

        allMeasurements.add(estimate.pose);

        // If this pose is the best estimate override the previous best
        if (isBetterEstimate(estimate, bestEstimate)) {
          bestEstimate = estimate;
        }

      } catch (Exception e) {
        DataLogManager.log("Vision processing failed for a camera: " + e.getMessage());
      }
    }

    // If we have a best estimate, send it to the pose estimator
    if (bestEstimate != null) {
      swerve.addVisionMeasurement(
          bestEstimate.pose,
          Utils.fpgaToCurrentTime(bestEstimate.timestampSeconds),
          getDynamicStdDevs(bestEstimate));
      finalMeasurementPub.set(bestEstimate.pose);
    } else {
      finalMeasurementPub.set(Pose2d.kZero);
    }

    // Log all measurements
    visionMeasurementsPub.set(allMeasurements.toArray(new Pose2d[0]));
    visibleTagsPub.set(visibleTags.toArray(new Pose3d[0]));
  }

  @Override
  public boolean setDesiredState(SubsystemState desiredState) {
    if (!super.setDesiredState(desiredState)) return false;

    int[] tagIds = desiredState.getTagIds().stream().mapToInt(Integer::intValue).toArray();
    for (String camera : cameras) {
      // Update limelight overrides for tag IDs
      LimelightHelpers.SetFiducialIDFiltersOverride(camera, tagIds);
    }

    return true;
  }

  private CameraResult processCamera(String camera) {
    var stateCopy = swerve.getStateCopy();

    // Tell the LL the robots orientation (MegaTag 2)
    LimelightHelpers.SetRobotOrientation(
        camera, stateCopy.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    // Get le pose
    LimelightHelpers.PoseEstimate pose =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
    if (pose == null || pose.tagCount == 0) {
      return new CameraResult(camera, null);
    }

    return new CameraResult(camera, pose);
  }

  public void setThrottle(int throttle) {
    for (String camera : cameras) {
      // El throttel
      LimelightHelpers.SetThrottle(camera, throttle);
    }
  }

  private static boolean isBetterEstimate(
      LimelightHelpers.PoseEstimate newEstimate, LimelightHelpers.PoseEstimate currentBest) {
    if (currentBest == null) {
      return newEstimate.avgTagDist < 4.125;
    }
    // The closer the tags the better the measurement
    return newEstimate.avgTagDist < currentBest.avgTagDist;
  }

  /** Computes dynamic standard deviations based on tag count and distance.
   * Taken from YAGSL.
   * */
  private Matrix<N3, N1> getDynamicStdDevs(LimelightHelpers.PoseEstimate estimate) {
    if (estimate.tagCount == 0) {
      return VecBuilder.fill(0.5, 0.5, 0.5);
    }

    LimelightHelpers.RawFiducial[] fiducials = estimate.rawFiducials;
    double avgDist = 0.0;
    for (LimelightHelpers.RawFiducial f : fiducials) {
      avgDist += f.distToCamera;
    }
    avgDist /= fiducials.length;

    double factor = 0.9 + (avgDist * avgDist / 30.0);

    double x = 0.5 * factor;
    double y = 0.5 * factor;
    double theta = estimate.isMegaTag2 ? Double.POSITIVE_INFINITY : 0.5 * factor;

    return VecBuilder.fill(x, y, theta);
  }

  /** Small record class for camera processing results. */
  private record CameraResult(String camera, LimelightHelpers.PoseEstimate estimate) {}
}
