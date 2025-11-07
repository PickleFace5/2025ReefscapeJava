package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
  // Hardware Objects
  private final TalonFX intakeTalon;
  private final CANrange intakeCANRange;

  // Control Requests
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  // Inputs
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrentAmps;
  private final StatusSignal<Boolean> coralDetected;

  // Connection Debouncers
  private final Debouncer intakeConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOTalonFX() {
    intakeTalon = new TalonFX(Constants.CanIDs.INTAKE_TALON);
    intakeCANRange = new CANrange(Constants.CanIDs.INTAKE_CANRANGE);

    // Configs
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.Slot0.kP = Constants.IntakeConstants.GAINS.kP;
    intakeConfig.Slot0.kI = Constants.IntakeConstants.GAINS.kI;
    intakeConfig.Slot0.kD = Constants.IntakeConstants.GAINS.kD;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.Feedback.SensorToMechanismRatio = Constants.ElevatorConstants.GEAR_RATIO;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.SUPPLY_CURRENT;
    intakeConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = Constants.CanIDs.INTAKE_CANRANGE;
    intakeConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANrange;
    tryUntilOk(5, () -> intakeTalon.getConfigurator().apply(intakeConfig));

    var CANRangeConfig = new CANrangeConfiguration();
    CANRangeConfig.ProximityParams.ProximityThreshold = 0.1;
    tryUntilOk(5, () -> intakeCANRange.getConfigurator().apply(CANRangeConfig));

    // Create status signals
    intakeVelocity = intakeTalon.getVelocity();
    intakeAppliedVolts = intakeTalon.getMotorVoltage();
    intakeCurrentAmps = intakeTalon.getStatorCurrent();
    coralDetected = intakeCANRange.getIsDetected();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, intakeVelocity, intakeAppliedVolts, intakeCurrentAmps, coralDetected);
    intakeTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var intakeStatus =
        BaseStatusSignal.refreshAll(intakeVelocity, intakeAppliedVolts, intakeCurrentAmps);

    inputs.intakeConnected = intakeConnectedDebounce.calculate(intakeStatus.isOK());
    inputs.intakeVelocityRadPerSec = Units.rotationsToRadians(intakeVelocity.getValueAsDouble());
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeCurrentAmps = intakeCurrentAmps.getValueAsDouble();
    inputs.sensorSensed = coralDetected.getValue();
  }

  @Override
  public void setIntakeOpenLoop(double output, boolean ignoreLimits) {
    intakeTalon.setControl(
        dutyCycleRequest.withOutput(output).withIgnoreHardwareLimits(ignoreLimits));
  }
}
