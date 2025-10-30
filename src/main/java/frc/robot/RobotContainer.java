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
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController functions = new CommandXboxController(1);

  private final SwerveSubsystem drivetrain;
  private final ClimberSubsystem climber;
  private final PivotSubsystem pivot;
  private final ElevatorSubsystem elevator;
  private final FunnelSubsystem funnel;
  private final IntakeSubsystem intake;
  private final VisionSubsystem vision;
  private final Superstructure superstructure;
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
  private final double maxAngularRate = Math.toRadians(360);

  private final FieldCentric fieldCentric;
  private final RobotCentric robotCentric;
  private final DriverAssist driverAssist;
  private final SwerveDriveBrake brake;
  private final PointWheelsAt point;

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Subsystems
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
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                fieldCentric
                    .withVelocityX(-driver.getLeftY() * maxSpeed)
                    .withVelocityY(-driver.getLeftX() * maxSpeed)
                    .withRotationalRate(-driver.getRightX() * maxAngularRate)));

    driver
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    robotCentric
                        .withVelocityX(-driver.getLeftY() * maxSpeed)
                        .withVelocityY(-driver.getLeftX() * maxSpeed)
                        .withRotationalRate(-driver.getRightX() * maxAngularRate)));

    driver
        .rightBumper()
        .whileTrue(intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_OUTPUT))
        .onFalse(intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

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

    driver.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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
                    .alongWith(
                        intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_INTAKE)))
            .onFalse(intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_HOLD));
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
                intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.FUNNEL_INTAKE)))
        .onFalse(
            Commands.parallel(
                superstructure.setGoalCommand(Superstructure.Goal.DEFAULT),
                intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD)));
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

    // Climbing
    functions
        .povLeft()
        .or(functions.povUpLeft())
        .or(functions.povDownLeft())
        .onTrue(
            Commands.parallel(
                climber.setDesiredStateCommand(ClimberSubsystem.SubsystemState.CLIMB_OUT),
                superstructure.setGoalCommand(Superstructure.Goal.CLIMBING)))
        .onFalse(climber.setDesiredStateCommand(ClimberSubsystem.SubsystemState.STOP));

    functions
        .povRight()
        .or(functions.povUpRight())
        .or(functions.povDownRight())
        .onTrue(
            Commands.parallel(
                climber.setDesiredStateCommand(ClimberSubsystem.SubsystemState.CLIMB_IN),
                superstructure.setGoalCommand(Superstructure.Goal.CLIMBING)))
        .onFalse(climber.setDesiredStateCommand(ClimberSubsystem.SubsystemState.STOP));

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
        "Hold", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.HOLD));
    NamedCommands.registerCommand(
        "Coral Intake", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_INTAKE));
    NamedCommands.registerCommand(
        "Coral Output", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.CORAL_OUTPUT));
    NamedCommands.registerCommand(
        "Algae Intake", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_INTAKE));
    NamedCommands.registerCommand(
        "Algae Output", intake.setDesiredStateCommand(IntakeSubsystem.SubsystemState.ALGAE_OUTPUT));
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
      autoChooser.addOption(name, new PathPlannerAuto(name, false));
      autoChooser.addOption(name + " (Mirrored)", new PathPlannerAuto(name, true));
    }
    autoChooser.setDefaultOption("None", Commands.none());
    autoChooser.addOption(
        "Basic Leave",
        drivetrain.applyRequest(() -> robotCentric.withVelocityX(1)).withTimeout(1.0));
    autoChooser.onChange((cmd) -> setCorrectSwervePosition());
    SmartDashboard.putData("Selected Auto", autoChooser);
  }

  private void setCorrectSwervePosition() {
    Object selected = autoChooser.getSelected();
    if (selected instanceof PathPlannerAuto auto) {
      Pose2d startingPose = auto.getStartingPose();
      drivetrain.resetPose(flipPoseIfNeeded(startingPose));
      drivetrain.resetRotation(
          startingPose.getRotation().plus(drivetrain.getOperatorForwardDirection()));
    }
  }

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

  public void setVisionThrottle(int throttle) {
    vision.setThrottle(throttle);
  }
}
