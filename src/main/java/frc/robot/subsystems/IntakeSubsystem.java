package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import java.util.EnumMap;

public class IntakeSubsystem extends StateSubsystem<IntakeSubsystem.SubsystemState> {

  public enum SubsystemState {
    HOLD,
    ALGAE_HOLD,
    CORAL_INTAKE,
    FUNNEL_INTAKE,
    CORAL_OUTPUT,
    ALGAE_INTAKE,
    ALGAE_OUTPUT,
    L1_OUTPUT
  }

  private static final CANrangeConfiguration CANRANGE_CONFIG = new CANrangeConfiguration();

  static {
    CANRANGE_CONFIG.ProximityParams.ProximityThreshold = 0.1;
  }

  private static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();

  static {
    MOTOR_CONFIG.Slot0.kP = Constants.IntakeConstants.GAINS.kP;
    MOTOR_CONFIG.Slot0.kI = Constants.IntakeConstants.GAINS.kI;
    MOTOR_CONFIG.Slot0.kD = Constants.IntakeConstants.GAINS.kD;

    MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    MOTOR_CONFIG.Feedback.SensorToMechanismRatio = Constants.ElevatorConstants.GEAR_RATIO;

    MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.SUPPLY_CURRENT;
  }

  private static final HardwareLimitSwitchConfigs LIMIT_SWITCH_CONFIG =
      new HardwareLimitSwitchConfigs();

  static {
    LIMIT_SWITCH_CONFIG.ForwardLimitRemoteSensorID = Constants.CanIDs.INTAKE_CANRANGE;
    LIMIT_SWITCH_CONFIG.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANrange;
  }

  private static final EnumMap<SubsystemState, IntakeConfig> STATE_CONFIGS =
      new EnumMap<>(SubsystemState.class);

  static {
    STATE_CONFIGS.put(SubsystemState.HOLD, new IntakeConfig(0, false));
    STATE_CONFIGS.put(
        SubsystemState.ALGAE_HOLD, new IntakeConfig(Constants.IntakeConstants.ALGAE_HOLD, true));
    STATE_CONFIGS.put(
        SubsystemState.CORAL_INTAKE,
        new IntakeConfig(Constants.IntakeConstants.CORAL_INTAKE_SPEED, false));
    STATE_CONFIGS.put(
        SubsystemState.FUNNEL_INTAKE,
        new IntakeConfig(Constants.IntakeConstants.FUNNEL_INTAKE_SPEED, false));
    STATE_CONFIGS.put(
        SubsystemState.CORAL_OUTPUT,
        new IntakeConfig(Constants.IntakeConstants.CORAL_OUTPUT_SPEED, true));
    STATE_CONFIGS.put(
        SubsystemState.ALGAE_INTAKE,
        new IntakeConfig(Constants.IntakeConstants.ALGAE_INTAKE_SPEED, false));
    STATE_CONFIGS.put(
        SubsystemState.ALGAE_OUTPUT,
        new IntakeConfig(Constants.IntakeConstants.ALGAE_OUTPUT_SPEED, true));
    STATE_CONFIGS.put(
        SubsystemState.L1_OUTPUT,
        new IntakeConfig(Constants.IntakeConstants.L1_OUTPUT_SPEED, true));
  }

  private final TalonFX intakeMotor;
  private final CANrange canrange;
  private final DutyCycleOut velocityRequest;

  public IntakeSubsystem() {
    super("Intake", SubsystemState.HOLD);

    intakeMotor = new TalonFX(Constants.CanIDs.INTAKE_TALON);
    if (!Utils.isSimulation()) {
      MOTOR_CONFIG.HardwareLimitSwitch = LIMIT_SWITCH_CONFIG;
    }
    intakeMotor.getConfigurator().apply(MOTOR_CONFIG);

    canrange = new CANrange(Constants.CanIDs.INTAKE_CANRANGE);
    canrange.getConfigurator().apply(CANRANGE_CONFIG);

    velocityRequest = new DutyCycleOut(0);
  }

  @Override
  public boolean setDesiredState(SubsystemState desiredState) {
    if (!super.setDesiredState(desiredState)) return false;

    IntakeConfig config = STATE_CONFIGS.getOrDefault(desiredState, new IntakeConfig(0, false));
    velocityRequest.Output = config.output;
    velocityRequest.IgnoreHardwareLimits = config.ignoreLimits;

    intakeMotor.setControl(velocityRequest);
    return true;
  }

  public Command setDesiredStateCommand(SubsystemState state) {
    return new InstantCommand(() -> setDesiredState(state), this);
  }

  public boolean hasCoral() {
    return intakeMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  private record IntakeConfig(double output, boolean ignoreLimits) {}
}
