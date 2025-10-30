package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * DriverAssist request automatically aligns the robot to a target pose while allowing velocity
 * input.
 */
public class DriverAssist implements SwerveRequest {

  private double velocityX = 0.0; // forward/back
  private double velocityY = 0.0; // left/right
  private double deadband = 0.0; // velocity deadband
  private double rotationalDeadband = 0.0; // rotational deadband

  private SwerveModule.DriveRequestType driveRequestType = SwerveModule.DriveRequestType.Velocity;
  private SwerveModule.SteerRequestType steerRequestType = SwerveModule.SteerRequestType.Position;

  private boolean desaturateWheelSpeeds = true;

  private ForwardPerspectiveValue forwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

  private PhoenixPIDController translationController = new PhoenixPIDController(0.0, 0.0, 0.0);

  private Pose2d targetPose = new Pose2d();

  private FieldCentricFacingAngle fieldCentricFacingAngle = new FieldCentricFacingAngle();

  private PhoenixPIDController headingController = fieldCentricFacingAngle.HeadingController;

  public Pose2d getTargetPose() {
    return targetPose;
  }

  public void setTargetPose(Pose2d pose) {
    targetPose = pose;
  }

  public DriverAssist withTargetPose(Pose2d pose) {
    setTargetPose(pose);
    return this;
  }

  public DriverAssist withVelocityX(double velX) {
    this.velocityX = velX;
    return this;
  }

  public DriverAssist withVelocityY(double velY) {
    this.velocityY = velY;
    return this;
  }

  public DriverAssist withDriveRequestType(SwerveModule.DriveRequestType type) {
    this.driveRequestType = type;
    return this;
  }

  public DriverAssist withSteerRequestType(SwerveModule.SteerRequestType type) {
    this.steerRequestType = type;
    return this;
  }

  public DriverAssist withTranslationPID(double p, double i, double d) {
    translationController.setPID(p, i, d);
    return this;
  }

  public DriverAssist withHeadingPID(double p, double i, double d) {
    headingController.setPID(p, i, d);
    return this;
  }

  public DriverAssist withDeadband(double deadband) {
    this.deadband = deadband;
    return this;
  }

  public DriverAssist withRotationalDeadband(double rDeadband) {
    this.rotationalDeadband = rDeadband;
    return this;
  }

  // --- Utility ---
  private double applyDeadband(double value, double deadband) {
    return Math.abs(value) >= deadband ? value : 0.0;
  }

  @Override
  public StatusCode apply(
      SwerveDrivetrain.SwerveControlParameters parameters,
      SwerveModule<?, ?, ?>... modulesToApply) {
    var targetRot = targetPose.getRotation();
    if (forwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
      targetRot = targetRot.plus(parameters.operatorForwardDirection);
    }

    // Compute Y error rotated to robot frame
    double yError =
        (parameters.currentPose.getX() * -targetRot.getSin()
                + parameters.currentPose.getY() * targetRot.getCos())
            - (targetPose.getX() * -targetRot.getSin() + targetPose.getY() * targetRot.getCos());

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      yError *= -1.0;
    }

    // Rotate robot relative velocity to field-centric
    double clamped =
        Math.max(
            -1.5,
            Math.min(translationController.calculate(yError, 0.0, parameters.timestamp), 1.5));
    double velY = applyDeadband(clamped, deadband);

    Translation2d fieldRelativeVelocity =
        new Translation2d(velocityX * targetRot.getCos() + velocityY * targetRot.getSin(), velY)
            .rotateBy(targetRot);

    return fieldCentricFacingAngle
        .withVelocityX(fieldRelativeVelocity.getX())
        .withVelocityY(fieldRelativeVelocity.getY())
        .withTargetDirection(targetRot)
        .withDeadband(0.0)
        .withRotationalDeadband(rotationalDeadband)
        .withDriveRequestType(driveRequestType)
        .withSteerRequestType(steerRequestType)
        .withDesaturateWheelSpeeds(desaturateWheelSpeeds)
        .withForwardPerspective(forwardPerspective)
        .apply(parameters, modulesToApply);
  }
}
