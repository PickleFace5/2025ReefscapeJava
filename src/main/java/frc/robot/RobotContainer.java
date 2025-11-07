package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.funnel.FunnelIO;
import frc.robot.subsystems.funnel.FunnelIOSim;
import frc.robot.subsystems.funnel.FunnelIOTalonFX;
import frc.robot.subsystems.funnel.FunnelSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOTalonFX;
import frc.robot.subsystems.vision.*;
import java.io.File;
import java.util.Map;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController functions =
      new CommandXboxController(RobotBase.isReal() ? 1 : 0);

  private final DriveSubsystem drivetrain;
  private final ClimberSubsystem climber;
  private final PivotSubsystem pivot;
  private final ElevatorSubsystem elevator;
  private final FunnelSubsystem funnel;
  private final IntakeSubsystem intake;
  private final VisionSubsystem vision;
  private final Superstructure superstructure;

  private LoggedDashboardChooser<Command> autoChooser;

  public static SwerveDriveSimulation swerveDriveSimulation =
      new SwerveDriveSimulation(
          DriveSubsystem.driveTrainSimulationConfig, new Pose2d(3, 3, Rotation2d.kZero));

  static {
    if (!RobotBase.isReal()) {
      SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
    }
  }

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain =
            new DriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {});
        climber = new ClimberSubsystem(new ClimberIOTalonFX());
        pivot = new PivotSubsystem(new PivotIOTalonFX());
        elevator = new ElevatorSubsystem(new ElevatorIOTalonFX());
        funnel = new FunnelSubsystem(new FunnelIOTalonFX());
        intake = new IntakeSubsystem(new IntakeIOTalonFX());
        vision =
            new VisionSubsystem(
                drivetrain::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.frontCamera, drivetrain::getRotation),
                new VisionIOLimelight(VisionConstants.frontLeftCamera, drivetrain::getRotation),
                new VisionIOLimelight(VisionConstants.frontRightCamera, drivetrain::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain =
            new DriveSubsystem(
                new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
                new ModuleIOSim(swerveDriveSimulation.getModules()[0]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[1]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[2]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[3]),
                (robotPose) -> swerveDriveSimulation.setSimulationWorldPose(robotPose));
        drivetrain.setPose(swerveDriveSimulation.getSimulatedDriveTrainPose());
        climber = new ClimberSubsystem(new ClimberIOSim());
        pivot = new PivotSubsystem(new PivotIOSim());
        elevator = new ElevatorSubsystem(new ElevatorIOSim());
        funnel = new FunnelSubsystem(new FunnelIOSim());
        intake = new IntakeSubsystem(new IntakeIOSim(swerveDriveSimulation));
        vision =
            new VisionSubsystem(
                drivetrain::addVisionMeasurement,
                VisionIOPhotonVisionSim.ofLimelight4(
                    VisionConstants.frontCamera,
                    VisionConstants.robotToFrontCam,
                    swerveDriveSimulation::getSimulatedDriveTrainPose),
                VisionIOPhotonVisionSim.ofLimelight3A(
                    VisionConstants.frontLeftCamera,
                    VisionConstants.robotToFrontLeftCam,
                    swerveDriveSimulation::getSimulatedDriveTrainPose),
                VisionIOPhotonVisionSim.ofLimelight3A(
                    VisionConstants.frontRightCamera,
                    VisionConstants.robotToFrontRightCam,
                    swerveDriveSimulation::getSimulatedDriveTrainPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});
        climber = new ClimberSubsystem(new ClimberIO() {});
        pivot = new PivotSubsystem(new PivotIO() {});
        elevator = new ElevatorSubsystem(new ElevatorIO() {});
        funnel = new FunnelSubsystem(new FunnelIO() {});
        intake = new IntakeSubsystem(new IntakeIO() {});
        vision =
            new VisionSubsystem(
                drivetrain::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        break;
    }

    superstructure = new Superstructure(drivetrain, pivot, elevator, funnel, climber, intake);

    setupControllerBindings();
    setupPathPlanner();
  }

  private void setupControllerBindings() {

    // DRIVE CONTROLLER
    drivetrain.setDefaultCommand(
        DriveCommands.fieldRelative(
            drivetrain,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.robotRelative(
                drivetrain,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

    driver
        .rightBumper()
        .whileTrue(intake.setDesiredStateCommand(IntakeSubsystem.State.CORAL_OUTPUT))
        .onFalse(intake.setDesiredStateCommand(IntakeSubsystem.State.HOLD));

    driver.a().whileTrue(DriveCommands.brakeWithX(drivetrain));

    driver
        .leftTrigger()
        .whileTrue(
            DriveCommands.alignToClosestReefBranch(
                drivetrain,
                DriveCommands.BranchSide.LEFT,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX()));

    driver
        .rightTrigger()
        .whileTrue(
            DriveCommands.alignToClosestReefBranch(
                drivetrain,
                DriveCommands.BranchSide.RIGHT,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX()));

    // Reset gyro to 0 when start button is pressed.
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drivetrain.setPose(
                            new Pose2d(drivetrain.getPose().getTranslation(), new Rotation2d())),
                    drivetrain)
                .ignoringDisable(true));

    // FUNCTION CONTROLLER
    // Map all triggers to respective superstructure goal
    Map<Trigger, Superstructure.Goal> goalBindings =
        Map.ofEntries(
            Map.entry(functions.y(), Superstructure.Goal.L4_CORAL),
            Map.entry(functions.x(), Superstructure.Goal.L3_CORAL),
            Map.entry(functions.b(), Superstructure.Goal.L2_CORAL),
            Map.entry(functions.a(), Superstructure.Goal.DEFAULT),
            Map.entry(functions.y().and(functions.start()), Superstructure.Goal.NET),
            Map.entry(functions.x().and(functions.start()), Superstructure.Goal.L3_ALGAE),
            Map.entry(functions.b().and(functions.start()), Superstructure.Goal.L2_ALGAE),
            Map.entry(functions.a().and(functions.start()), Superstructure.Goal.PROCESSOR),
            Map.entry(functions.leftStick(), Superstructure.Goal.L1_CORAL));

    // Create all simple triggers
    for (Map.Entry<Trigger, Superstructure.Goal> entry : goalBindings.entrySet()) {
      Trigger condition = entry.getKey();
      Superstructure.Goal goal = entry.getValue();

      if (goal == Superstructure.Goal.L3_ALGAE
          || goal == Superstructure.Goal.NET
          || goal == Superstructure.Goal.L2_ALGAE
          || goal == Superstructure.Goal.PROCESSOR) {
        condition
            .whileTrue(
                superstructure
                    .setGoalCommand(goal)
                    .alongWith(intake.setDesiredStateCommand(IntakeSubsystem.State.ALGAE_INTAKE)))
            .onFalse(intake.setDesiredStateCommand(IntakeSubsystem.State.ALGAE_HOLD));
      } else {
        condition.onTrue(superstructure.setGoalCommand(goal));
      }
    }

    // Intaking
    functions
        .leftBumper()
        .onTrue(
            Commands.parallel(
                superstructure.setGoalCommand(Superstructure.Goal.FUNNEL),
                intake.setDesiredStateCommand(IntakeSubsystem.State.FUNNEL_INTAKE)))
        .onFalse(
            Commands.parallel(
                superstructure.setGoalCommand(Superstructure.Goal.DEFAULT),
                intake.setDesiredStateCommand(IntakeSubsystem.State.HOLD)));
    functions
        .leftBumper()
        .and(functions.back())
        .whileTrue(
            Commands.parallel(
                superstructure.setGoalCommand(Superstructure.Goal.FLOOR),
                intake.setDesiredStateCommand(IntakeSubsystem.State.CORAL_INTAKE)))
        .onFalse(
            Commands.parallel(
                superstructure.setGoalCommand(Superstructure.Goal.DEFAULT),
                intake.setDesiredStateCommand(IntakeSubsystem.State.HOLD)));

    // Climbing
    functions
        .povLeft()
        .or(functions.povUpLeft())
        .or(functions.povDownLeft())
        .onTrue(
            Commands.parallel(
                climber.setStateCommand(ClimberSubsystem.State.CLIMB_OUT),
                superstructure.setGoalCommand(Superstructure.Goal.CLIMBING)))
        .onFalse(climber.setStateCommand(ClimberSubsystem.State.STOP));

    functions
        .povRight()
        .or(functions.povUpRight())
        .or(functions.povDownRight())
        .onTrue(
            Commands.parallel(
                climber.setStateCommand(ClimberSubsystem.State.CLIMB_IN),
                superstructure.setGoalCommand(Superstructure.Goal.CLIMBING)))
        .onFalse(climber.setStateCommand(ClimberSubsystem.State.STOP));

    functions.povUp().onTrue(superstructure.setGoalCommand(Superstructure.Goal.FINISH));
  }

  private void setupPathPlanner() {
    // Register named commands for superstructure and intake
    NamedCommands.registerCommand(
        "Default", superstructure.setGoalCommand(Superstructure.Goal.DEFAULT));
    NamedCommands.registerCommand(
        "L4 Coral", superstructure.setGoalCommand(Superstructure.Goal.L4_CORAL));
    NamedCommands.registerCommand(
        "L3 Coral", superstructure.setGoalCommand(Superstructure.Goal.L3_CORAL));
    NamedCommands.registerCommand(
        "L2 Coral", superstructure.setGoalCommand(Superstructure.Goal.L2_CORAL));
    NamedCommands.registerCommand(
        "L1 Coral", superstructure.setGoalCommand(Superstructure.Goal.L1_CORAL));
    NamedCommands.registerCommand(
        "L2 Algae", superstructure.setGoalCommand(Superstructure.Goal.L2_ALGAE));
    NamedCommands.registerCommand(
        "L3 Algae", superstructure.setGoalCommand(Superstructure.Goal.L3_ALGAE));
    NamedCommands.registerCommand(
        "Processor", superstructure.setGoalCommand(Superstructure.Goal.PROCESSOR));
    NamedCommands.registerCommand("Net", superstructure.setGoalCommand(Superstructure.Goal.NET));
    NamedCommands.registerCommand(
        "Funnel", superstructure.setGoalCommand(Superstructure.Goal.FUNNEL));
    NamedCommands.registerCommand(
        "Floor", superstructure.setGoalCommand(Superstructure.Goal.FLOOR));

    NamedCommands.registerCommand(
        "Hold", intake.setDesiredStateCommand(IntakeSubsystem.State.HOLD));
    NamedCommands.registerCommand(
        "Coral Intake", intake.setDesiredStateCommand(IntakeSubsystem.State.CORAL_INTAKE));
    NamedCommands.registerCommand(
        "Coral Output", intake.setDesiredStateCommand(IntakeSubsystem.State.CORAL_OUTPUT));
    NamedCommands.registerCommand(
        "Algae Intake", intake.setDesiredStateCommand(IntakeSubsystem.State.ALGAE_INTAKE));
    NamedCommands.registerCommand(
        "Algae Output", intake.setDesiredStateCommand(IntakeSubsystem.State.ALGAE_OUTPUT));
    NamedCommands.registerCommand(
        "Funnel Intake",
        intake
            .setDesiredStateCommand(IntakeSubsystem.State.FUNNEL_INTAKE)
            .repeatedly()
            .until(() -> (intake.hasCoral() || Utils.isSimulation())));

    // Auto chooser
    autoChooser = new LoggedDashboardChooser<>("Selected Auto");
    File autosFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
    for (File file : autosFolder.listFiles()) {
      if (!file.getName().endsWith(".auto") || file.getName().equals(".DS_Store")) continue;
      String name = file.getName().replace(".auto", "");
      autoChooser.addOption(name, new PathPlannerAuto(name, false));
      autoChooser.addOption(name + " (Mirrored)", new PathPlannerAuto(name, true));
    }
    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption(
        "Basic Leave",
        drivetrain
            .run(() -> DriveCommands.robotRelative(drivetrain, () -> 1, () -> 0, () -> 0))
            .withTimeout(1.0));
    autoChooser.addOption(
        "Feedforward Characterization",
        drivetrain.run(() -> DriveCommands.feedforwardCharacterization(drivetrain)));
    autoChooser.addOption(
        "Wheel Radius Characterization",
        drivetrain.run(() -> DriveCommands.wheelRadiusCharacterization(drivetrain)));
  }

  public void readyRobotForMatch() {
    if (autoChooser.get() instanceof PathPlannerAuto auto) {
      drivetrain.setPose(auto.getStartingPose());
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
