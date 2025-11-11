package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest.*;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.DriverAssist;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.io.File;
import java.util.Map;
import java.util.Optional;

public class RobotContainer {
  // Setup controller ports
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController functions = new CommandXboxController(1);

  // Subsystems
  private final SwerveSubsystem drivetrain;
  private final ClimberSubsystem climber;
  private final PivotSubsystem pivot;
  private final ElevatorSubsystem elevator;
  private final FunnelSubsystem funnel;
  private final IntakeSubsystem intake;
  private final VisionSubsystem vision;
  private final Superstructure superstructure;

  // Drivetrain max speeds for teleop control
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
  private final double maxAngularRate = Math.toRadians(360);

  // Swerve requests
  private final FieldCentric fieldCentric;
  private final RobotCentric robotCentric;
  private final DriverAssist driverAssist;
  private final SwerveDriveBrake brake;
  private final PointWheelsAt point;

  // Elastic Auto Chooser
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Initialize subsystems
    drivetrain = TunerConstants.createDrivetrain();
    climber = new ClimberSubsystem();
    pivot = new PivotSubsystem();
    elevator = new ElevatorSubsystem();
    funnel = new FunnelSubsystem();
    intake = new IntakeSubsystem();
    vision =
        new VisionSubsystem(
            drivetrain,
            Constants.VisionConstants.FRONT_RIGHT,
            Constants.VisionConstants.FRONT_CENTER,
            Constants.VisionConstants.FRONT_LEFT);
    superstructure =
        new Superstructure(drivetrain, pivot, elevator, funnel, vision, climber, intake);

    // Swerve Requests
    fieldCentric =
        new FieldCentric()
            .withDeadband(0)
            .withRotationalDeadband(0)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    robotCentric =
        new RobotCentric()
            .withDeadband(0)
            .withRotationalDeadband(0)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    driverAssist =
        new DriverAssist()
            .withDeadband(maxSpeed * 0.01)
            .withRotationalDeadband(maxAngularRate * 0.02)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position)
            .withTranslationPID(
                Constants.AutoAlignConstants.TRANSLATION_P,
                Constants.AutoAlignConstants.TRANSLATION_I,
                Constants.AutoAlignConstants.TRANSLATION_D)
            .withHeadingPID(
                Constants.AutoAlignConstants.HEADING_P,
                Constants.AutoAlignConstants.HEADING_I,
                Constants.AutoAlignConstants.HEADING_D);

    brake = new SwerveDriveBrake();
    point = new PointWheelsAt();

    setupControllerBindings();
    setupPathPlanner();
  }

  private void setupControllerBindings() {

    // DRIVE CONTROLLER
    // FieldCentric as default control
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                fieldCentric
                    .withVelocityX(-driver.getLeftY() * maxSpeed)
                    .withVelocityY(-driver.getLeftX() * maxSpeed)
                    .withRotationalRate(-driver.getRightX() * maxAngularRate)));

    // Left bumper: robot centric (mainly for ground intaking)
    driver
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    robotCentric
                        .withVelocityX(-driver.getLeftY() * maxSpeed)
                        .withVelocityY(-driver.getLeftX() * maxSpeed)
                        .withRotationalRate(-driver.getRightX() * maxAngularRate)));

    // Right bumper: drop coral
    driver
        .rightBumper()
        .whileTrue(intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_OUTPUT))
        .onFalse(intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD));

    // A: Lock wheels in an X formation (mainly used for tuning)
    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // B: Point wheels in the given direction (used for tuning)
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // Right Trigger: Auto align to the nearest branch on the right
    driver
        .rightTrigger()
        .onTrue(
            drivetrain.runOnce(
                () ->
                    driverAssist.setTargetPose(
                        drivetrain.getClosestBranch(SwerveSubsystem.BranchSide.RIGHT))))
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    driverAssist
                        .withVelocityX(-driver.getLeftY() * maxSpeed)
                        .withVelocityY(-driver.getLeftX() * maxSpeed)));

    // Left trigger: Auto align to the nearest branch on the left
    driver
        .leftTrigger()
        .onTrue(
            drivetrain.runOnce(
                () ->
                    driverAssist.setTargetPose(
                        drivetrain.getClosestBranch(SwerveSubsystem.BranchSide.LEFT))))
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    driverAssist
                        .withVelocityX(-driver.getLeftY() * maxSpeed)
                        .withVelocityY(-driver.getLeftX() * maxSpeed)));

    // Reset gyro manually (in case of offset on startup)
    driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // FUNCTION CONTROLLER
    // Map all triggers to respective superstructure goal
    Map<Trigger, Superstructure.Goal> goalBindings =
        Map.ofEntries(
            Map.entry(functions.y(), Superstructure.Goal.L4_CORAL), // Y: L4 Coral
            Map.entry(functions.x(), Superstructure.Goal.L3_CORAL), // X: L3 Coral
            Map.entry(functions.b(), Superstructure.Goal.L2_CORAL), // B: L2 Coral
            Map.entry(functions.a(), Superstructure.Goal.DEFAULT), // A: Default (startup state)
            Map.entry(functions.y().and(functions.start()), Superstructure.Goal.NET), // Y + start: Net scoring
            Map.entry(functions.x().and(functions.start()), Superstructure.Goal.L3_ALGAE), // X + start: L3 Algae
            Map.entry(functions.b().and(functions.start()), Superstructure.Goal.L2_ALGAE), // B + start: L2 Algae
            Map.entry(functions.a().and(functions.start()), Superstructure.Goal.PROCESSOR), // A + start: Processor
            Map.entry(functions.leftStick(), Superstructure.Goal.L1_CORAL)); // L3 (press left joystick): L1 Coral

    // Create triggers for function controller
    for (Map.Entry<Trigger, Superstructure.Goal> entry : goalBindings.entrySet()) {
      Trigger condition = entry.getKey();
      Superstructure.Goal goal = entry.getValue();

      // If algae related, set the intake accordingly
      if (goal == Superstructure.Goal.L3_ALGAE
          || goal == Superstructure.Goal.NET
          || goal == Superstructure.Goal.L2_ALGAE
          || goal == Superstructure.Goal.PROCESSOR) {
        condition
            .whileTrue(
                superstructure
                    .setGoalCommand(goal)
                    .alongWith(
                        intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_INTAKE)))
            .onFalse(intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_HOLD));
      } else {
        // Otherwise just change the superstructure state
        condition.onTrue(superstructure.setGoalCommand(goal));
      }
    }

    // Funnel intake
    functions
        .leftBumper()
        .onTrue(
            Commands.parallel(
                superstructure.setGoalCommand(Superstructure.Goal.FUNNEL),
                intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.FUNNEL_INTAKE)))
        .onFalse(
            Commands.parallel(
                superstructure.setGoalCommand(Superstructure.Goal.DEFAULT),
                intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD)));

    // Ground intake
    functions
        .leftBumper()
        .and(functions.back())
        .whileTrue(
            Commands.parallel(
                superstructure.setGoalCommand(Superstructure.Goal.FLOOR),
                intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_INTAKE)))
        .onFalse(
            Commands.parallel(
                superstructure.setGoalCommand(Superstructure.Goal.DEFAULT),
                intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD)));

    // Climber out
    functions
        .povLeft()
        .or(functions.povUpLeft())
        .or(functions.povDownLeft())
        .onTrue(
            Commands.parallel(
                climber.setDesiredStateCommand(ClimberSubsystem.SubsystemState.CLIMB_OUT),
                superstructure.setGoalCommand(Superstructure.Goal.CLIMBING)))
        .onFalse(climber.setDesiredStateCommand(ClimberSubsystem.SubsystemState.STOP));

    // Climber in
    functions
        .povRight()
        .or(functions.povUpRight())
        .or(functions.povDownRight())
        .onTrue(
            Commands.parallel(
                climber.setDesiredStateCommand(ClimberSubsystem.SubsystemState.CLIMB_IN),
                superstructure.setGoalCommand(Superstructure.Goal.CLIMBING)))
        .onFalse(climber.setDesiredStateCommand(ClimberSubsystem.SubsystemState.STOP));

    // "Finish" (move pivot into cage to balance CoG)
    functions.povUp().onTrue(superstructure.setGoalCommand(Superstructure.Goal.FINISH));
  }

  private void setupPathPlanner() {
    // Register named commands for superstructure and intake
    // (These bind into commands for PathPlanner paths)
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
        "Hold", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD));
    NamedCommands.registerCommand(
        "Coral Intake", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_INTAKE));
    NamedCommands.registerCommand(
        "Coral Output", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_OUTPUT));
    NamedCommands.registerCommand(
        "Algae Intake", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_INTAKE));
    NamedCommands.registerCommand(
        "Algae Output", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_OUTPUT));
    // Funnel intake runs until we detect coral (or if we're simulating for testing)
    NamedCommands.registerCommand(
        "Funnel Intake",
        intake
            .setDesiredStateCommand(IntakeSubsystem.SubsystemState.FUNNEL_INTAKE)
            .repeatedly()
            .until(() -> (intake.hasCoral() || Utils.isSimulation())));

    // Auto chooser
    autoChooser = new SendableChooser<>();
    File autosFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
    for (File file : autosFolder.listFiles()) {
      if (!file.getName().endsWith(".auto") || file.getName().equals(".DS_Store")) continue;
      String name = file.getName().replace(".auto", "");

      // Create both normal and mirrored auto (normal autos on the left, mirrored autos on the right)
      autoChooser.addOption(name, new PathPlannerAuto(name, false));
      autoChooser.addOption(name + " (Mirrored)", new PathPlannerAuto(name, true));
    }
    // Backup autos (none does... nothing... Basic Leave drives forward at 1 m/s for 1 second)
    autoChooser.setDefaultOption("None", Commands.none());
    autoChooser.addOption(
        "Basic Leave",
        drivetrain.applyRequest(() -> robotCentric.withVelocityX(1)).withTimeout(1.0));

    // When we detect the auto selected has changed, reset the drivetrain pose
    autoChooser.onChange((cmd) -> setCorrectSwervePosition());

    SmartDashboard.putData("Selected Auto", autoChooser);
  }

  /// Resets drivetrain pose to the selected auto's start.
  /// This is to allow the vision to correct for an incidental translational offset before the match begins
  private void setCorrectSwervePosition() {
    Object selected = autoChooser.getSelected();
    if (selected instanceof PathPlannerAuto auto) {
      Pose2d startingPose = auto.getStartingPose();
      drivetrain.resetPose(flipPoseIfNeeded(startingPose));
      drivetrain.resetRotation(
          startingPose.getRotation().plus(drivetrain.getOperatorForwardDirection()));
    }
  }

  /// Returns a flipped pose if on the red alliance (used in setCorrectSwervePosition)
  private static Pose2d flipPoseIfNeeded(Pose2d pose) {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      double flippedX = Constants.FIELD_LAYOUT.getFieldLength() - pose.getX();
      double flippedY = Constants.FIELD_LAYOUT.getFieldWidth() - pose.getY();
      Rotation2d flippedRotation = pose.getRotation().plus(Rotation2d.k180deg);
      return new Pose2d(flippedX, flippedY, flippedRotation);
    }
    return pose;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /// Sets LimeLight throttle (see docs for details)
  public void setVisionThrottle(int throttle) {
    vision.setThrottle(throttle);
  }
}
