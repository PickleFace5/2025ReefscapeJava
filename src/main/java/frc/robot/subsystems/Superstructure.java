package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  // Behold my wall of text
  public enum Goal {
    DEFAULT(PivotSubsystem.State.STOW, ElevatorSubsystem.State.DEFAULT, FunnelSubsystem.State.DOWN),
    L4_CORAL(
        PivotSubsystem.State.HIGH_SCORING, ElevatorSubsystem.State.L4, FunnelSubsystem.State.DOWN),
    L3_CORAL(PivotSubsystem.State.L3_CORAL, ElevatorSubsystem.State.L3, FunnelSubsystem.State.DOWN),
    L2_CORAL(PivotSubsystem.State.L2_CORAL, ElevatorSubsystem.State.L2, FunnelSubsystem.State.DOWN),
    L1_CORAL(
        PivotSubsystem.State.LOW_SCORING, ElevatorSubsystem.State.L1, FunnelSubsystem.State.DOWN),
    L2_ALGAE(
        PivotSubsystem.State.ALGAE_INTAKE,
        ElevatorSubsystem.State.L2_ALGAE,
        FunnelSubsystem.State.DOWN),
    L3_ALGAE(
        PivotSubsystem.State.ALGAE_INTAKE,
        ElevatorSubsystem.State.L3_ALGAE,
        FunnelSubsystem.State.DOWN),
    PROCESSOR(
        PivotSubsystem.State.PROCESSOR_SCORING,
        ElevatorSubsystem.State.PROCESSOR,
        FunnelSubsystem.State.DOWN),
    NET(PivotSubsystem.State.NET_SCORING, ElevatorSubsystem.State.NET, FunnelSubsystem.State.DOWN),
    CLIMBING(
        PivotSubsystem.State.AVOID_CLIMBER,
        ElevatorSubsystem.State.DEFAULT,
        FunnelSubsystem.State.DOWN),
    FUNNEL(
        PivotSubsystem.State.FUNNEL_INTAKE,
        ElevatorSubsystem.State.DEFAULT,
        FunnelSubsystem.State.UP),
    FLOOR(
        PivotSubsystem.State.GROUND_INTAKE,
        ElevatorSubsystem.State.DEFAULT,
        FunnelSubsystem.State.DOWN),
    FINISH(
        PivotSubsystem.State.FUNNEL_INTAKE,
        ElevatorSubsystem.State.DEFAULT,
        FunnelSubsystem.State.UP);

    public final PivotSubsystem.State pivotState;
    public final ElevatorSubsystem.State elevatorState;
    public final FunnelSubsystem.State funnelState;

    Goal(
        PivotSubsystem.State pivotState,
        ElevatorSubsystem.State elevatorState,
        FunnelSubsystem.State funnelState) {
      this.pivotState = pivotState;
      this.elevatorState = elevatorState;
      this.funnelState = funnelState;
    }
  }

  private final DriveSubsystem drive;
  private final PivotSubsystem pivot;
  private final ElevatorSubsystem elevator;
  private final FunnelSubsystem funnel;
  private final ClimberSubsystem climber;
  private final IntakeSubsystem intake;

  @AutoLogOutput(key = "Superstructure/Goal")
  private Goal currentGoal = Goal.DEFAULT;

  @AutoLogOutput(key = "Superstructure/Current Game Piece")
  private static Pose3d currentGamePiece = new Pose3d();

  private PivotSubsystem.State desiredPivotState;
  private ElevatorSubsystem.State desiredElevatorState;

  public Superstructure(
      DriveSubsystem drive,
      PivotSubsystem pivot,
      ElevatorSubsystem elevator,
      FunnelSubsystem funnel,
      ClimberSubsystem climber,
      IntakeSubsystem intake) {
    setName("Superstructure");
    this.drive = drive;
    this.pivot = pivot;
    this.elevator = elevator;
    this.funnel = funnel;
    this.climber = climber;
    this.intake = intake;

    setGoal(currentGoal);
  }

  @Override
  public void periodic() {
    if (!DriverStation.isDisabled()) {

      // Avoid eating the intake lol
      if (pivot.isOutsideElevator()) {
        elevator.setState(desiredElevatorState);
      }
      if (pivot.getState() == PivotSubsystem.State.AVOID_ELEVATOR
          && pivot.isOutsideElevator()
          && (elevator.isAtSetpoint()
              || desiredPivotState.position.getRotations()
                  < Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE)) {
        pivot.setState(desiredPivotState);
      }

      if (climber.getPosition() > Constants.ClimberConstants.CLIMB_FULL_THRESHOLD
          && climber.getState() == ClimberSubsystem.State.CLIMB_IN) {
        climber.setState(ClimberSubsystem.State.CLIMB_IN_FULL);
      }
    }

    // Update component poses
    Pose3d[] elevatorPoses = elevator.getComponentPoses();
    Pose3d pivotPose = pivot.getComponentPose(elevatorPoses[1]); // Feed in carriage pose for height
    Logger.recordOutput(
        "Superstructure/Components",
        funnel.getComponentPose(),
        elevatorPoses[0],
        elevatorPoses[1],
        pivotPose,
        climber.getComponentPose());

    // If we have a game piece, display it in the robot
    if (intake.hasCoral()) {
      Pose3d robotPose =
          new Pose3d(
              RobotBase.isReal()
                  ? drive.getPose()
                  : RobotContainer.swerveDriveSimulation.getSimulatedDriveTrainPose());

      Transform3d coralOffset =
          new Transform3d(
              // Origin to bumper + Coral length - random number to fix my math being wrong
              new Translation3d(0.134856 + 0.301625 - 0.055, 0, 0),
              // Angle offset when arm is straight sideways
              new Rotation3d(0.0, Units.degreesToRadians(-29.20274), 0.0));

      currentGamePiece =
          robotPose.transformBy(pivotPose.minus(new Pose3d())).transformBy(coralOffset);
    } else {
      currentGamePiece = new Pose3d();
    }
  }

  /// SIMULATION ONLY
  public static boolean canFunnelIntake() {
    if (RobotBase.isReal()) return false;

    // Check if we're at a coral station (rough bounding boxes for each corner of the field)
    AbstractDriveTrainSimulation driveSimulation = RobotContainer.swerveDriveSimulation;
    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    boolean atFunnel;
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red) {
      // Red alliance
      atFunnel =
          robotPose.getX() > 16
              && ((robotPose.getY() < 1.4
                      && MathUtil.isNear(127.25, robotPose.getRotation().getDegrees(), 5))
                  || (robotPose.getY() > 6.6
                      && MathUtil.isNear(-124, robotPose.getRotation().getDegrees(), 5)));
    } else {
      // Blue alliance
      atFunnel =
          robotPose.getX() < 2
              && ((robotPose.getY() < 1.4
                      && MathUtil.isNear(53.06, robotPose.getRotation().getDegrees(), 5))
                  || (robotPose.getY() > 6.5
                      && MathUtil.isNear(-52.6, robotPose.getRotation().getDegrees(), 5)));
    }
    return atFunnel;
  }

  public static void attemptCoralScore() {
    if (RobotBase.isReal()) return;
    AbstractDriveTrainSimulation driveSimulation = RobotContainer.swerveDriveSimulation;
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new ReefscapeCoralOnFly(
                // Obtain robot position from drive simulation
                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                new Translation2d(0.46, 0),
                // Obtain robot speed from drive simulation
                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                // Obtain robot facing from drive simulation
                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                // The height at which the coral is ejected
                Meters.of(currentGamePiece.getZ()),
                // The initial speed of the coral
                MetersPerSecond.of(-1),
                // The coral is ejected vertically downwards
                Degrees.of(100)));
  }

  private void setGoal(Goal goal) {
    this.currentGoal = goal;

    boolean safetyChecks = shouldEnableSafetyChecks(goal.pivotState, goal.elevatorState);

    desiredPivotState = goal.pivotState;
    if (safetyChecks) {
      pivot.setState(PivotSubsystem.State.AVOID_ELEVATOR);
    } else {
      pivot.setState(goal.pivotState);
    }

    desiredElevatorState = goal.elevatorState;
    if (goal.pivotState != null && safetyChecks) {
      elevator.setState(ElevatorSubsystem.State.IDLE);
    } else {
      elevator.setState(goal.elevatorState);
    }

    funnel.setState(goal.funnelState);
  }

  private boolean shouldEnableSafetyChecks(
      PivotSubsystem.State pivotState, ElevatorSubsystem.State elevatorState) {
    if (elevatorState == elevator.getState()) return false;
    return !(pivot.getState().position.getRotations()
            < Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE
        && pivotState.position.getRotations() < Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE);
  }

  public Command setGoalCommand(Goal goal) {
    return new InstantCommand(() -> setGoal(goal), this);
  }
}
