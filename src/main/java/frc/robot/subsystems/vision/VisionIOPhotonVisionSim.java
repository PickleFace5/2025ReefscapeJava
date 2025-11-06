package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;

  public static VisionIOPhotonVisionSim ofLimelight4(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    return new VisionIOPhotonVisionSim(
        name,
        robotToCamera,
        poseSupplier,
        new SimCameraProperties()
            .setFPS(40)
            .setCalibration(1280, 960, Rotation2d.fromDegrees(82))
            .setExposureTimeMs(6.2));
  }

  public static VisionIOPhotonVisionSim ofLimelight3A(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    return new VisionIOPhotonVisionSim(
        name,
        robotToCamera,
        poseSupplier,
        new SimCameraProperties()
            .setFPS(50)
            .setCalibration(640, 480, Rotation2d.fromDegrees(54.5))
            .setExposureTimeMs(18.87));
  }

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d robotToCamera,
      Supplier<Pose2d> poseSupplier,
      SimCameraProperties cameraProperties) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(Constants.FIELD_LAYOUT);
    }

    // Add sim camera (disable camera streams, we don't really care tbh)
    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProperties.setAvgLatencyMs(15));
    cameraSim.enableRawStream(false);
    cameraSim.enableProcessedStream(false);
    cameraSim.setMaxSightRange(4.25); // Can only see at most half the field length
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
