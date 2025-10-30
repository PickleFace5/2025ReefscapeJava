package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import java.util.*;
import java.util.function.Supplier;

/**
 * Extends Phoenix 6's {@link TunerSwerveDrivetrain} and integrates: - SysId routines
 * (translation/steer/rotation) - PathPlanner AutoBuilder - Alliance perspective switching - Reef
 * branch pose targeting helpers - Simulation updates
 */
public class SwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem {
  private static final double SIM_LOOP_PERIOD = 0.005;

  private double lastSimTime;

  private static final Rotation2d BLUE_ALLIANCE_ROTATION = Rotation2d.kZero;
  private static final Rotation2d RED_ALLIANCE_ROTATION = Rotation2d.k180deg;
  private boolean hasAppliedPerspective = false;

  /** SysId characterization requests */
  private final SwerveRequest.SysIdSwerveTranslation translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();

  private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /** SysId routines */
  private SysIdRoutine sysIdTranslation;

  private SysIdRoutine sysIdSteer;
  private SysIdRoutine sysIdRotation;
  private SysIdRoutine sysIdToApply;

  /** Request used during PathPlanner path following */
  private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /** Enum describing which branch side to target */
  public enum BranchSide {
    LEFT,
    RIGHT
  }

  // Predefined reef branch target poses (Blue alliance, mirrored for Red)
  private static final Pose2d[] BLUE_LEFT_BRANCHES = {
    new Pose2d(3.091, 4.181, Rotation2d.kZero),
    new Pose2d(3.656, 2.916, Rotation2d.fromDegrees(60)),
    new Pose2d(5.023, 2.772, Rotation2d.fromDegrees(120)),
    new Pose2d(5.850, 3.851, Rotation2d.k180deg),
    new Pose2d(5.347, 5.134, Rotation2d.fromDegrees(240)),
    new Pose2d(3.932, 5.302, Rotation2d.fromDegrees(300))
  };

  private static final Pose2d[] BLUE_RIGHT_BRANCHES = {
    new Pose2d(3.091, 3.863, Rotation2d.kZero),
    new Pose2d(3.956, 2.748, Rotation2d.fromDegrees(60)),
    new Pose2d(5.323, 2.928, Rotation2d.fromDegrees(120)),
    new Pose2d(5.862, 4.187, Rotation2d.k180deg),
    new Pose2d(5.047, 5.290, Rotation2d.fromDegrees(240)),
    new Pose2d(3.668, 5.110, Rotation2d.fromDegrees(300))
  };

  private static final Pose2d[] RED_LEFT_BRANCHES = mirrorPoses(BLUE_LEFT_BRANCHES);
  private static final Pose2d[] RED_RIGHT_BRANCHES = mirrorPoses(BLUE_RIGHT_BRANCHES);

  private static Pose2d[] mirrorPoses(Pose2d[] blueTargets) {
    Pose2d[] mirrored = new Pose2d[blueTargets.length];
    double fieldLength = Constants.FIELD_LAYOUT.getFieldLength();
    double fieldWidth = Constants.FIELD_LAYOUT.getFieldWidth();
    for (int i = 0; i < blueTargets.length; i++) {
      Pose2d pose = blueTargets[i];
      mirrored[i] =
          new Pose2d(
              fieldLength - pose.getX(),
              fieldWidth - pose.getY(),
              pose.getRotation().plus(Rotation2d.k180deg));
    }
    return mirrored;
  }

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("Telemetry");
  private static final StructPublisher<Pose2d> posePub =
      table.getStructTopic("current_pose", Pose2d.struct).publish();
  private static final StructPublisher<ChassisSpeeds> speedsPub =
      table.getStructTopic("chassis_speeds", ChassisSpeeds.struct).publish();
  private static final DoublePublisher odomFreqPub =
      table.getDoubleTopic("odometry_frequency").publish();
  private static final StructArrayPublisher<SwerveModuleState> moduleStatesPub =
      table.getStructArrayTopic("module_states", SwerveModuleState.struct).publish();
  private static final StructArrayPublisher<SwerveModuleState> moduleTargetsPub =
      table.getStructArrayTopic("module_targets", SwerveModuleState.struct).publish();
  private static final StructPublisher<Pose2d> autoTargetPub =
      table.getStructTopic("auto_target", Pose2d.struct).publish();
  private static final StructArrayPublisher<Pose2d> autoPathPub =
      table.getStructArrayTopic("auto_path", Pose2d.struct).publish();
  private static final StructPublisher<Pose2d> closestBranchPub =
      table.getStructTopic("Closest Branch", Pose2d.struct).publish();

  static {
    PathPlannerLogging.setLogTargetPoseCallback(autoTargetPub::set);
    // PathPlannerLogging.setLogActivePathCallback((poses) -> autoPathPub.set(());
  }

  // --- Constructors ---
  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);

    getPigeon2().reset();

    setupSysId();
    configureAutoBuilder();
    if (Utils.isSimulation()) startSimThread();
  }

  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odomStdDevs,
      Matrix<N3, N1> visionStdDevs,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, odomStdDevs, visionStdDevs, modules);
    setupSysId();
    configureAutoBuilder();
    if (Utils.isSimulation()) startSimThread();
  }

  /** Configure SysId routines */
  private void setupSysId() {
    sysIdTranslation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                edu.wpi.first.units.Units.Volts.of(4),
                null,
                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> setControl(translationCharacterization.withVolts(volts)), null, this));

    sysIdSteer =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                edu.wpi.first.units.Units.Volts.of(7),
                null,
                state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                volts -> setControl(steerCharacterization.withVolts(volts)), null, this));

    sysIdRotation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                edu.wpi.first.units.Units.Volts.of(Math.PI / 6)
                    .per(edu.wpi.first.units.Units.Second),
                edu.wpi.first.units.Units.Volts.of(Math.PI),
                null,
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                output -> {
                  setControl(
                      rotationCharacterization.withRotationalRate(
                          output.in(edu.wpi.first.units.Units.Volts)));
                  SignalLogger.writeDouble(
                      "Rotational_Rate", output.in(edu.wpi.first.units.Units.Volts));
                },
                null,
                this));

    sysIdToApply = sysIdSteer;
  }

  /** Configure PathPlanner AutoBuilder */
  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose,
          this::resetPose,
          () -> getState().Speeds,
          (speeds, feedforwards) ->
              setControl(
                  pathApplyRobotSpeeds
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              new PIDConstants(2.75, 0, 0), new PIDConstants(2.75, 0, 0)),
          config,
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this);
    } catch (Exception e) {
      DriverStation.reportError("Failed to configure AutoBuilder", e.getStackTrace());
    }
  }

  /** Apply a control request continuously */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /** SysId commands */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdToApply.dynamic(direction);
  }

  /** Get the closest reef branch pose based on alliance and side */
  public Pose2d getClosestBranch(BranchSide side) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Pose2d[] targets =
        (alliance == Alliance.Red)
            ? (side == BranchSide.LEFT ? RED_LEFT_BRANCHES : RED_RIGHT_BRANCHES)
            : (side == BranchSide.LEFT ? BLUE_LEFT_BRANCHES : BLUE_RIGHT_BRANCHES);

    Pose2d robotPose = getState().Pose;

    Pose2d closestBranch =
        Arrays.stream(targets)
            .min(Comparator.comparingDouble(p -> getDistanceToLine(robotPose, p)))
            .orElse(targets[0]);

    closestBranchPub.set(closestBranch);
    return closestBranch;
  }

  /** Compute distance between robot and target branch line */
  private static double getDistanceToLine(Pose2d robot, Pose2d target) {
    double slope = Math.tan(target.getRotation().getRadians());
    double x =
        (robot.getX()
                + slope * slope * target.getX()
                - slope * target.getY()
                + slope * robot.getY())
            / (slope * slope + 1);
    double y = slope * (x - target.getX()) + target.getY();

    Pose2d possible = new Pose2d(x, y, target.getRotation());
    double reefX =
        (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
            ? 4.4735
            : Constants.FIELD_LAYOUT.getFieldLength() - 4.4735;

    if ((target.getX() - reefX <= 0) == (x - reefX <= 0))
      return robot.getTranslation().getDistance(possible.getTranslation());
    return Double.POSITIVE_INFINITY;
  }

  @Override
  public void periodic() {
    if (!hasAppliedPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              alliance -> {
                setOperatorPerspectiveForward(
                    alliance == Alliance.Red ? RED_ALLIANCE_ROTATION : BLUE_ALLIANCE_ROTATION);
                hasAppliedPerspective = true;
              });
    }

    SwerveDriveState state = getStateCopy();
    posePub.set(state.Pose);
    odomFreqPub.set(1.0 / state.OdometryPeriod);
    moduleStatesPub.set(state.ModuleStates);
    moduleTargetsPub.set(state.ModuleTargets);
    speedsPub.set(state.Speeds);
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();
    Notifier simNotifier =
        new Notifier(
            () -> {
              double now = Utils.getCurrentTimeSeconds();
              double dt = now - lastSimTime;
              lastSimTime = now;
              updateSimState(dt, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }
}
