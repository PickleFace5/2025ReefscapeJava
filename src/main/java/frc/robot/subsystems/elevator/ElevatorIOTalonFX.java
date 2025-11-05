package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import java.util.Optional;

public class ElevatorIOTalonFX implements ElevatorIO {
  // Hardware objects
  private final TalonFX mainTalon;
  private final TalonFX followerTalon;
  private final CANdi limitSwitchCANdi;

  // Control requests
  private final DynamicMotionMagicVoltage positionRequest =
      new DynamicMotionMagicVoltage(
          0,
          Constants.ElevatorConstants.CRUISE_VELOCITY,
          Constants.ElevatorConstants.MM_UPWARD_ACCELERATION,
          Constants.ElevatorConstants.MM_JERK);
  private final DynamicMotionMagicVoltage brakeRequest =
      new DynamicMotionMagicVoltage(
          0,
          Constants.ElevatorConstants.CRUISE_VELOCITY,
          Constants.ElevatorConstants.MM_BRAKE_ACCELERATION,
          0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // Device inputs
  private final StatusSignal<Angle> mainPosition;
  private final StatusSignal<AngularVelocity> mainVelocity;
  private final StatusSignal<Voltage> mainAppliedVolts;
  private final StatusSignal<Current> mainCurrentAmps;

  // Connection debouncers
  private final Debouncer mainTalonConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerTalonConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer CANdiConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ElevatorIOTalonFX() {
    mainTalon = new TalonFX(Constants.CanIDs.LEFT_ELEVATOR_TALON);
    followerTalon = new TalonFX(Constants.CanIDs.RIGHT_ELEVATOR_TALON);
    limitSwitchCANdi = new CANdi(Constants.CanIDs.ELEVATOR_CANDI);

    // CANdi config
    CANdiConfiguration CANdiConfig = new CANdiConfiguration();
    tryUntilOk(5, () -> limitSwitchCANdi.getConfigurator().apply(CANdiConfig, 0.25));

    // Motor Config
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSlot0(Constants.ElevatorConstants.GAINS)
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(Constants.ElevatorConstants.GEAR_RATIO))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(
                        Constants.ElevatorConstants.MM_DOWNWARD_ACCELERATION)
                    .withMotionMagicCruiseVelocity(Constants.ElevatorConstants.CRUISE_VELOCITY))
            .withHardwareLimitSwitch(
                new HardwareLimitSwitchConfigs()
                    .withForwardLimitRemoteSensorID(Constants.CanIDs.ELEVATOR_CANDI)
                    .withForwardLimitSource(ForwardLimitSourceValue.RemoteCANdiS1)
                    .withForwardLimitAutosetPositionValue(Constants.ElevatorConstants.ELEVATOR_MAX)
                    .withForwardLimitAutosetPositionEnable(false)
                    .withReverseLimitRemoteSensorID(Constants.CanIDs.ELEVATOR_CANDI)
                    .withReverseLimitSource(ReverseLimitSourceValue.RemoteCANdiS2)
                    .withReverseLimitAutosetPositionValue(
                        Constants.ElevatorConstants.DEFAULT_POSITION)
                    .withReverseLimitAutosetPositionEnable(false));
    tryUntilOk(5, () -> mainTalon.getConfigurator().apply(motorConfig, 0.25));
    tryUntilOk(5, () -> followerTalon.getConfigurator().apply(motorConfig, 0.25));
    tryUntilOk(5, () -> mainTalon.setPosition(Constants.ElevatorConstants.DEFAULT_POSITION));

    // Create status signals
    mainPosition = mainTalon.getPosition();
    mainVelocity = mainTalon.getVelocity();
    mainAppliedVolts = mainTalon.getMotorVoltage();
    mainCurrentAmps = mainTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(250, mainPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, mainVelocity, mainAppliedVolts, mainCurrentAmps);
    ParentDevice.optimizeBusUtilizationForAll(mainTalon, followerTalon);

    // Setup initial control
    mainTalon.setControl(brakeRequest);
    followerTalon.setControl(new Follower(mainTalon.getDeviceID(), false));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Refresh all signals
    var talonStatus =
        BaseStatusSignal.refreshAll(mainPosition, mainVelocity, mainAppliedVolts, mainCurrentAmps);

    // Update inputs
    inputs.elevatorConnected =
        mainTalonConnectedDebounce.calculate(talonStatus.isOK())
            && followerTalonConnectedDebounce.calculate(followerTalon.isConnected());
    inputs.limitSwitchConnected = CANdiConnectedDebounce.calculate(limitSwitchCANdi.isConnected());
    inputs.positionRads = Units.rotationsToRadians(mainPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(mainVelocity.getValueAsDouble());
    inputs.appliedVolts = mainAppliedVolts.getValueAsDouble();
    inputs.currentAmps = mainCurrentAmps.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double output) {
    mainTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setPosition(Optional<Double> position) {
    if (position.isEmpty()) {
      brakeRequest.Position = mainTalon.getPosition().getValueAsDouble();
      mainTalon.setControl(brakeRequest);
    } else {
      if (mainTalon.getPosition().getValue().in(Rotations) < position.get()) {
        positionRequest.Acceleration = Constants.ElevatorConstants.MM_UPWARD_ACCELERATION;
      } else {
        positionRequest.Acceleration = Constants.ElevatorConstants.MM_DOWNWARD_ACCELERATION;
      }
      positionRequest.Position = position.get();
      mainTalon.setControl(positionRequest);
    }
  }
}
