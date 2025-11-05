package frc.robot.subsystems.funnel;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FunnelIOTalonFX implements FunnelIO {
  // Hardware objects
  private final TalonFX funnelTalon;

  // Control Request lol
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);

  // Inputs
  private final StatusSignal<Angle> funnelPosition;
  private final StatusSignal<AngularVelocity> funnelVelocity;
  private final StatusSignal<Voltage> funnelAppliedVolts;
  private final StatusSignal<Current> funnelCurrentAmps;

  // Connection Debouncer
  private final Debouncer funnelConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public FunnelIOTalonFX() {
    funnelTalon = new TalonFX(Constants.CanIDs.FUNNEL_TALON);

    // Configure motor
    var funnelConfig = new TalonFXConfiguration();
    funnelConfig.Feedback.SensorToMechanismRatio = Constants.FunnelConstants.GEAR_RATIO;
    funnelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    funnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    funnelConfig.Slot0.kP = Constants.FunnelConstants.GAINS.kP;
    funnelConfig.Slot0.kI = Constants.FunnelConstants.GAINS.kI;
    funnelConfig.Slot0.kD = Constants.FunnelConstants.GAINS.kD;
    funnelConfig.MotionMagic.MotionMagicAcceleration = Constants.FunnelConstants.MM_ACCELERATION;
    funnelConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.FunnelConstants.CRUISE_VELOCITY;
    funnelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    funnelConfig.CurrentLimits.SupplyCurrentLimit = Constants.FunnelConstants.SUPPLY_CURRENT;
    funnelConfig.CurrentLimits.SupplyCurrentLowerTime = 0;
    funnelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    funnelConfig.CurrentLimits.StatorCurrentLimit = Constants.FunnelConstants.STATOR_CURRENT;
    tryUntilOk(5, () -> funnelTalon.getConfigurator().apply(funnelConfig, 0.25));

    // Create status signals
    funnelPosition = funnelTalon.getPosition();
    funnelVelocity = funnelTalon.getVelocity();
    funnelAppliedVolts = funnelTalon.getMotorVoltage();
    funnelCurrentAmps = funnelTalon.getStatorCurrent();

    // Configure periodic frames
    funnelPosition.setUpdateFrequency(250);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, funnelVelocity, funnelAppliedVolts, funnelCurrentAmps);
    funnelTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    // Refresh signals
    var funnelStatus =
        BaseStatusSignal.refreshAll(
            funnelPosition, funnelVelocity, funnelAppliedVolts, funnelCurrentAmps);

    // Update inputs
    inputs.funnelConnected = funnelConnectedDebounce.calculate(funnelStatus.isOK());
    inputs.funnelPosition = Rotation2d.fromRotations(funnelPosition.getValueAsDouble());
    inputs.funnelVelocityRadPerSec = Units.rotationsToRadians(funnelVelocity.getValueAsDouble());
    inputs.funnelAppliedVolts = funnelAppliedVolts.getValueAsDouble();
    inputs.funnelCurrentAmps = funnelCurrentAmps.getValueAsDouble();
  }

  @Override
  public void setFunnelPosition(Rotation2d rotation) {
    funnelTalon.setControl(positionRequest.withPosition(rotation.getRotations()));
  }
}
