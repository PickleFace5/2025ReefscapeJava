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
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.EnumMap;
import java.util.Map;

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

  // Subsystems
  private final SwerveSubsystem drivetrain;
  private final PivotSubsystem pivot;
  private final ElevatorSubsystem elevator;
  private final FunnelSubsystem funnel;
  private final VisionSubsystem vision;
  private final ClimberSubsystem climber;
  private final IntakeSubsystem intake;

  private Goal currentGoal = Goal.DEFAULT;

  // Desired state for safety checks
  private PivotSubsystem.SubsystemState desiredPivotState;
  private ElevatorSubsystem.SubsystemState desiredElevatorState;

  private final Map<Goal, GoalStates> goalToStates = new EnumMap<>(Goal.class);

  private record GoalStates(
      PivotSubsystem.SubsystemState pivotState,
      ElevatorSubsystem.SubsystemState elevatorState,
      FunnelSubsystem.SubsystemState funnelState,
      VisionSubsystem.SubsystemState visionState) {}

  // Publishers
  static NetworkTable table = NetworkTableInstance.getDefault().getTable("Superstructure");
  private static final StringPublisher currentGoalPub =
      table.getStringTopic("Current Goal").publish();
  private static final StructArrayPublisher<Pose3d> componentPoses =
      table.getStructArrayTopic("Components", Pose3d.struct).publish();

  public Superstructure(
      SwerveSubsystem drivetrain,
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
            PivotSubsystem.SubsystemState.STOW,
            ElevatorSubsystem.SubsystemState.DEFAULT,
            FunnelSubsystem.SubsystemState.DOWN,
            VisionSubsystem.SubsystemState.ALL_ESTIMATES));
    goalToStates.put(
        Goal.L4_CORAL,
        new GoalStates(
            PivotSubsystem.SubsystemState.HIGH_SCORING,
            ElevatorSubsystem.SubsystemState.L4,
            FunnelSubsystem.SubsystemState.DOWN,
            VisionSubsystem.SubsystemState.REEF_ESTIMATES));
    goalToStates.put(
        Goal.L3_CORAL,
        new GoalStates(
            PivotSubsystem.SubsystemState.L3_CORAL,
            ElevatorSubsystem.SubsystemState.L3,
            FunnelSubsystem.SubsystemState.DOWN,
            VisionSubsystem.SubsystemState.REEF_ESTIMATES));
    goalToStates.put(
        Goal.L2_CORAL,
        new GoalStates(
            PivotSubsystem.SubsystemState.L2_CORAL,
            ElevatorSubsystem.SubsystemState.L2,
            FunnelSubsystem.SubsystemState.DOWN,
            VisionSubsystem.SubsystemState.REEF_ESTIMATES));
    goalToStates.put(
        Goal.L1_CORAL,
        new GoalStates(
            PivotSubsystem.SubsystemState.LOW_SCORING,
            ElevatorSubsystem.SubsystemState.L1,
            FunnelSubsystem.SubsystemState.DOWN,
            VisionSubsystem.SubsystemState.REEF_ESTIMATES));
    goalToStates.put(
        Goal.L2_ALGAE,
        new GoalStates(
            PivotSubsystem.SubsystemState.ALGAE_INTAKE,
            ElevatorSubsystem.SubsystemState.L2_ALGAE,
            FunnelSubsystem.SubsystemState.DOWN,
            VisionSubsystem.SubsystemState.REEF_ESTIMATES));
    goalToStates.put(
        Goal.L3_ALGAE,
        new GoalStates(
            PivotSubsystem.SubsystemState.ALGAE_INTAKE,
            ElevatorSubsystem.SubsystemState.L3_ALGAE,
            FunnelSubsystem.SubsystemState.DOWN,
            VisionSubsystem.SubsystemState.REEF_ESTIMATES));
    goalToStates.put(
        Goal.PROCESSOR,
        new GoalStates(
            PivotSubsystem.SubsystemState.PROCESSOR_SCORING,
            ElevatorSubsystem.SubsystemState.PROCESSOR,
            FunnelSubsystem.SubsystemState.DOWN,
            VisionSubsystem.SubsystemState.ALL_ESTIMATES));
    goalToStates.put(
        Goal.NET,
        new GoalStates(
            PivotSubsystem.SubsystemState.NET_SCORING,
            ElevatorSubsystem.SubsystemState.NET,
            FunnelSubsystem.SubsystemState.DOWN,
            VisionSubsystem.SubsystemState.ALL_ESTIMATES));
    goalToStates.put(
        Goal.FUNNEL,
        new GoalStates(
            PivotSubsystem.SubsystemState.FUNNEL_INTAKE,
            ElevatorSubsystem.SubsystemState.DEFAULT,
            FunnelSubsystem.SubsystemState.UP,
            VisionSubsystem.SubsystemState.ALL_ESTIMATES));
    goalToStates.put(
        Goal.FLOOR,
        new GoalStates(
            PivotSubsystem.SubsystemState.GROUND_INTAKE,
            ElevatorSubsystem.SubsystemState.DEFAULT,
            FunnelSubsystem.SubsystemState.DOWN,
            VisionSubsystem.SubsystemState.ALL_ESTIMATES));
    goalToStates.put(
        Goal.CLIMBING,
        new GoalStates(
            PivotSubsystem.SubsystemState.AVOID_CLIMBER,
            ElevatorSubsystem.SubsystemState.DEFAULT,
            FunnelSubsystem.SubsystemState.DOWN,
            VisionSubsystem.SubsystemState.ALL_ESTIMATES));
    goalToStates.put(
        Goal.FINISH,
        new GoalStates(
            PivotSubsystem.SubsystemState.FUNNEL_INTAKE,
            ElevatorSubsystem.SubsystemState.DEFAULT,
            FunnelSubsystem.SubsystemState.UP,
            VisionSubsystem.SubsystemState.ALL_ESTIMATES));

    setGoal(currentGoal);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) return;

    // Unfreeze subsystems if safe
    if (elevator.isFrozen() && pivot.isOutsideElevator()) {
      elevator.unfreeze();
      elevator.setDesiredState(desiredElevatorState);
    }

    if (pivot.getCurrentState() == PivotSubsystem.SubsystemState.AVOID_ELEVATOR
        && pivot.isOutsideElevator()
        && (elevator.isAtSetpoint()
            || desiredPivotState.position < Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE)) {
      pivot.unfreeze();
      pivot.setDesiredState(desiredPivotState);
    }

    if (climber.getPosition() > Constants.ClimberConstants.CLIMB_FULL_THRESHOLD
        && climber.getCurrentState() == ClimberSubsystem.SubsystemState.CLIMB_IN) {
      climber.setDesiredState(ClimberSubsystem.SubsystemState.CLIMB_IN_FULL);
    }

    // Component poses for AdvantageScope fanciness
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

  private void setGoal(Goal goal) {
    this.currentGoal = goal;
    GoalStates states = goalToStates.get(goal);

    boolean safetyChecks = shouldEnableSafetyChecks(states.pivotState, states.elevatorState);

    // Set each subsystem state from goal
    if (states.pivotState != null) {
      desiredPivotState = states.pivotState;
      if (safetyChecks) {
        pivot.setDesiredState(PivotSubsystem.SubsystemState.AVOID_ELEVATOR);
        pivot.freeze();
      } else {
        pivot.setDesiredState(states.pivotState);
      }
    }

    if (states.elevatorState != null) {
      desiredElevatorState = states.elevatorState;
      if (states.pivotState != null && safetyChecks) {
        elevator.setDesiredState(ElevatorSubsystem.SubsystemState.IDLE);
        elevator.freeze();
      } else {
        elevator.setDesiredState(states.elevatorState);
      }
    }

    if (states.funnelState != null) funnel.setDesiredState(states.funnelState);
    if (states.visionState != null) vision.setDesiredState(states.visionState);

    // le publish goal
    currentGoalPub.set(goal.name());
  }

  ///  Enable safety checks if... that is true
  private boolean shouldEnableSafetyChecks(
      PivotSubsystem.SubsystemState pivotState, ElevatorSubsystem.SubsystemState elevatorState) {
    if (elevatorState == elevator.getCurrentState()) return false;
    return !(pivot.getCurrentState().position < Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE
        && pivotState.position < Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE);
  }

  public Command setGoalCommand(Goal goal) {
    return new InstantCommand(() -> setGoal(goal), this);
  }
}
