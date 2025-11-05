package frc.robot.subsystems.vision;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // Camera names, must match names configured on coprocessor
  public static String frontCamera = "limelight-front";
  // public static String backCamera = "limelight-back";
  public static String frontRightCamera = "limelight-fr";
  public static String frontLeftCamera = "limelight-fl";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToFrontCam =
      new Transform3d(
          0.2398808004, 0.0, 0.2009021128, new Rotation3d(0.0, degreesToRadians(-20.0), 0.0));
  public static Transform3d robotToBackCam =
      new Transform3d(0.030842, 0.0, 0.980425, new Rotation3d(0.0, 0.0, degreesToRadians(180)));
  public static Transform3d robotToFrontRightCam =
      new Transform3d(
          0.3055549388,
          -0.3039476268,
          0.2250257882,
          new Rotation3d(0.0, degreesToRadians(-15), degreesToRadians(-45)));
  public static Transform3d robotToFrontLeftCam =
      new Transform3d(
          0.3055549388,
          0.3039476268,
          0.2250257882,
          new Rotation3d(0.0, degreesToRadians(-15), degreesToRadians(45)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1-meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors = new double[] {1.0, 0.8, 0.8};

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
