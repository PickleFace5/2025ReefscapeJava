package frc.robot.subsystems.pivot;

import static edu.wpi.first.math.util.Units.rotationsToRadians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class PivotIOTalonFX implements PivotIO {
  // Hardware objects
  private final TalonFX pivotTalon;
  private final CANcoder cancoder;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
  private final StaticBrake brakeRequest = new StaticBrake();

  // Device inputs
  private final StatusSignal<Angle> pivotAbsolutePosition;
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotAppliedVolts;
  private final StatusSignal<Current> pivotCurrent;

  // Connection debouncers
  private final Debouncer pivotConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer encoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public PivotIOTalonFX() {
    pivotTalon = new TalonFX(Constants.CanIDs.PIVOT_TALON);
    cancoder = new CANcoder(Constants.CanIDs.PIVOT_CANCODER);

    // Encoder config
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = Constants.PivotConstants.CANCODER_OFFSET;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        Constants.PivotConstants.CANCODER_DISCONTINUITY;
    cancoder.getConfigurator().apply(encoderConfig);

    // Motor config
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Feedback.RotorToSensorRatio = Constants.PivotConstants.GEAR_RATIO;

    /// NOTE: With the cycloidal gearbox, the backlash may be mitigated to where this can be a
    /// FusedCANcoder for more accurate readings.
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    motorConfig.Feedback.FeedbackRemoteSensorID = Constants.CanIDs.PIVOT_CANCODER;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Slot0.kP = Constants.PivotConstants.GAINS.kP;
    motorConfig.Slot0.kI = Constants.PivotConstants.GAINS.kI;
    motorConfig.Slot0.kD = Constants.PivotConstants.GAINS.kD;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.PivotConstants.CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.PivotConstants.MM_ACCELERATION;
    pivotTalon.getConfigurator().apply(motorConfig);

    pivotTalon.setPosition(cancoder.getPosition().getValue());

    // Create status signals
    pivotAbsolutePosition = cancoder.getAbsolutePosition();
    pivotPosition = pivotTalon.getPosition();
    pivotVelocity = pivotTalon.getVelocity();
    pivotAppliedVolts = pivotTalon.getMotorVoltage();
    pivotCurrent = pivotTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(250, pivotPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pivotVelocity, pivotAppliedVolts, pivotCurrent, pivotAbsolutePosition);
    pivotTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    // Refresh all signals
    var talonStatus =
        BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, pivotAppliedVolts, pivotCurrent);
    var encoderStatus = BaseStatusSignal.refreshAll(pivotAbsolutePosition);

    // Update inputs
    inputs.pivotConnected = pivotConnectedDebounce.calculate(talonStatus.isOK());
    inputs.pivotEncoderConnected = encoderConnectedDebounce.calculate(encoderStatus.isOK());
    inputs.pivotAbsolutePosition =
        Rotation2d.fromRotations(pivotAbsolutePosition.getValueAsDouble());
    inputs.pivotPosition = Rotation2d.fromRotations(pivotPosition.getValueAsDouble());
    inputs.pivotVelocityRadPerSec = rotationsToRadians(pivotVelocity.getValueAsDouble());
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double output) {
    pivotTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    pivotTalon.setControl(positionRequest.withPosition(rotation.getRotations()));
  }
}
