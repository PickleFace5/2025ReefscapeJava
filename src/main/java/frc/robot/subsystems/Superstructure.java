package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.EnumMap;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {

  public enum Goal {
    DEFAULT,
    L4_CORAL,
    L3_CORAL,
    L2_CORAL,
    L1_CORAL,
    L2_ALGAE,
    L3_ALGAE,
    PROCESSOR,
    NET,
    CLIMBING,
    FUNNEL,
    FLOOR,
    FINISH
  }

  private final DriveSubsystem drivetrain;
  private final PivotSubsystem pivot;
  private final ElevatorSubsystem elevator;
  private final FunnelSubsystem funnel;
  private final VisionSubsystem vision;
  private final ClimberSubsystem climber;
  private final IntakeSubsystem intake;

  private Goal currentGoal = Goal.DEFAULT;

  private PivotSubsystem.State desiredPivotState;
  private ElevatorSubsystem.State desiredElevatorState;

  private final Map<Goal, GoalStates> goalToStates = new EnumMap<>(Goal.class);

  private record GoalStates(
      PivotSubsystem.State pivotState,
      ElevatorSubsystem.State elevatorState,
      FunnelSubsystem.State funnelState) {}

  static NetworkTable table =
      NetworkTableInstance.getDefault().getTable("AdvantageKit").getSubTable("Superstructure");
  private static final StringPublisher currentGoalPub =
      table.getStringTopic("Current Goal").publish();
  private static final StructArrayPublisher<Pose3d> componentPoses =
      table.getStructArrayTopic("Components", Pose3d.struct).publish();

  public Superstructure(
      DriveSubsystem drivetrain,
      PivotSubsystem pivot,
      ElevatorSubsystem elevator,
      FunnelSubsystem funnel,
      VisionSubsystem vision,
      ClimberSubsystem climber,
      IntakeSubsystem intake) {
    this.drivetrain = drivetrain;
    this.pivot = pivot;
    this.elevator = elevator;
    this.funnel = funnel;
    this.vision = vision;
    this.climber = climber;
    this.intake = intake;

    // Initialize goal map
    goalToStates.put(
        Goal.DEFAULT,
        new GoalStates(
            PivotSubsystem.State.STOW,
            ElevatorSubsystem.State.DEFAULT,
            FunnelSubsystem.State.DOWN));
    goalToStates.put(
        Goal.L4_CORAL,
        new GoalStates(
            PivotSubsystem.State.HIGH_SCORING,
            ElevatorSubsystem.State.L4,
            FunnelSubsystem.State.DOWN));
    goalToStates.put(
        Goal.L3_CORAL,
        new GoalStates(
            PivotSubsystem.State.L3_CORAL, ElevatorSubsystem.State.L3, FunnelSubsystem.State.DOWN));
    goalToStates.put(
        Goal.L2_CORAL,
        new GoalStates(
            PivotSubsystem.State.L2_CORAL, ElevatorSubsystem.State.L2, FunnelSubsystem.State.DOWN));
    goalToStates.put(
        Goal.L1_CORAL,
        new GoalStates(
            PivotSubsystem.State.LOW_SCORING,
            ElevatorSubsystem.State.L1,
            FunnelSubsystem.State.DOWN));
    goalToStates.put(
        Goal.L2_ALGAE,
        new GoalStates(
            PivotSubsystem.State.ALGAE_INTAKE,
            ElevatorSubsystem.State.L2_ALGAE,
            FunnelSubsystem.State.DOWN));
    goalToStates.put(
        Goal.L3_ALGAE,
        new GoalStates(
            PivotSubsystem.State.ALGAE_INTAKE,
            ElevatorSubsystem.State.L3_ALGAE,
            FunnelSubsystem.State.DOWN));
    goalToStates.put(
        Goal.PROCESSOR,
        new GoalStates(
            PivotSubsystem.State.PROCESSOR_SCORING,
            ElevatorSubsystem.State.PROCESSOR,
            FunnelSubsystem.State.DOWN));
    goalToStates.put(
        Goal.NET,
        new GoalStates(
            PivotSubsystem.State.NET_SCORING,
            ElevatorSubsystem.State.NET,
            FunnelSubsystem.State.DOWN));
    goalToStates.put(
        Goal.FUNNEL,
        new GoalStates(
            PivotSubsystem.State.FUNNEL_INTAKE,
            ElevatorSubsystem.State.DEFAULT,
            FunnelSubsystem.State.UP));
    goalToStates.put(
        Goal.FLOOR,
        new GoalStates(
            PivotSubsystem.State.GROUND_INTAKE,
            ElevatorSubsystem.State.DEFAULT,
            FunnelSubsystem.State.DOWN));
    goalToStates.put(
        Goal.CLIMBING,
        new GoalStates(
            PivotSubsystem.State.AVOID_CLIMBER,
            ElevatorSubsystem.State.DEFAULT,
            FunnelSubsystem.State.DOWN));
    goalToStates.put(
        Goal.FINISH,
        new GoalStates(
            PivotSubsystem.State.FUNNEL_INTAKE,
            ElevatorSubsystem.State.DEFAULT,
            FunnelSubsystem.State.UP));

    setGoal(currentGoal);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) return;

    // Unfreeze subsystems if safe
    if (pivot.isOutsideElevator()) {
      elevator.setState(desiredElevatorState);
    }

    if (pivot.getState() == PivotSubsystem.State.AVOID_ELEVATOR
        && pivot.isOutsideElevator()
        && (elevator.isAtSetpoint()
            || desiredPivotState.position.getRotations()
                < Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE)) {
      // pivot.unfreeze()
      pivot.setState(desiredPivotState);
    }

    if (climber.getPosition() > Constants.ClimberConstants.CLIMB_FULL_THRESHOLD
        && climber.getState() == ClimberSubsystem.State.CLIMB_IN) {
      climber.setState(ClimberSubsystem.State.CLIMB_IN_FULL);
    }

    Pose3d[] elevatorPoses = elevator.getComponentPoses();
    componentPoses.set(
        new Pose3d[] {
          funnel.getComponentPose(),
          elevatorPoses[0],
          elevatorPoses[1],
          pivot.getComponentPose(elevatorPoses[1]), // Feed in carriage pose for height
          climber.getComponentPose()
        });
  }

  @AutoLogOutput(key = "Superstructure/Components")
  public Pose3d[] getComponents() {
    Pose3d[] elevatorPoses = elevator.getComponentPoses();
    return new Pose3d[] {
      funnel.getComponentPose(),
      elevatorPoses[0],
      elevatorPoses[1],
      pivot.getComponentPose(elevatorPoses[1]), // Feed in carriage pose for height
      climber.getComponentPose()
    };
  }

  private void setGoal(Goal goal) {
    this.currentGoal = goal;
    GoalStates states = goalToStates.get(goal);

    boolean safetyChecks = shouldEnableSafetyChecks(states.pivotState, states.elevatorState);

    if (states.pivotState != null) {
      desiredPivotState = states.pivotState;
      if (safetyChecks) {
        pivot.setState(PivotSubsystem.State.AVOID_ELEVATOR);
        // pivot.freeze();
      } else {
        pivot.setState(states.pivotState);
      }
    }

    if (states.elevatorState != null) {
      desiredElevatorState = states.elevatorState;
      if (states.pivotState != null && safetyChecks) {
        elevator.setState(ElevatorSubsystem.State.IDLE);
      } else {
        elevator.setState(states.elevatorState);
      }
    }

    if (states.funnelState != null) funnel.setState(states.funnelState);

    currentGoalPub.set(goal.name());
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
