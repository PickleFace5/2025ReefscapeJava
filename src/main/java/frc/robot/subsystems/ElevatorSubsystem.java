package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ElevatorSubsystem extends StateSubsystem<ElevatorSubsystem.SubsystemState> {

  public enum SubsystemState {
    IDLE(null),
    DEFAULT(Constants.ElevatorConstants.DEFAULT_POSITION),
    L1(Constants.ElevatorConstants.L1_SCORE_POSITION),
    L2(Constants.ElevatorConstants.L2_SCORE_POSITION),
    L3(Constants.ElevatorConstants.L3_SCORE_POSITION),
    L4(Constants.ElevatorConstants.L4_SCORE_POSITION),
    L2_ALGAE(Constants.ElevatorConstants.L2_ALGAE_POSITION),
    L3_ALGAE(Constants.ElevatorConstants.L3_ALGAE_POSITION),
    NET(Constants.ElevatorConstants.NET_SCORE_POSITION),
    PROCESSOR(Constants.ElevatorConstants.PROCESSOR_SCORE_POSITION);

    public final Double position;

    SubsystemState(Double position) {
      this.position = position;
    }
  }

  private final TalonFX masterMotor;
  private final TalonFX followerMotor;
  private final CANdi candi;

  private final DynamicMotionMagicVoltage positionRequest;
  private final DynamicMotionMagicVoltage brakeRequest;
  private final VoltageOut sysIdRequest;

  private SysIdRoutine sysIdRoutine;
  private boolean atSetpoint = true;

  public ElevatorSubsystem() {
    super("Elevator", SubsystemState.DEFAULT);

    // CANdi
    candi = new CANdi(Constants.CanIDs.ELEVATOR_CANDI);
    CANdiConfiguration candiConfig = new CANdiConfiguration();
    candi.getConfigurator().apply(candiConfig);

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
                    .withMotionMagicCruiseVelocity(Constants.ElevatorConstants.CRUISE_VELOCITY));

    // Limit Switch Config
    HardwareLimitSwitchConfigs limitSwitch =
        new HardwareLimitSwitchConfigs()
            .withForwardLimitRemoteSensorID(Constants.CanIDs.ELEVATOR_CANDI)
            .withForwardLimitSource(ForwardLimitSourceValue.RemoteCANdiS1)
            .withForwardLimitAutosetPositionValue(Constants.ElevatorConstants.ELEVATOR_MAX)
            .withForwardLimitAutosetPositionEnable(false)
            .withReverseLimitRemoteSensorID(Constants.CanIDs.ELEVATOR_CANDI)
            .withReverseLimitSource(ReverseLimitSourceValue.RemoteCANdiS2)
            .withReverseLimitAutosetPositionValue(Constants.ElevatorConstants.DEFAULT_POSITION)
            .withReverseLimitAutosetPositionEnable(false);

    // Master and follower motors
    masterMotor = new TalonFX(Constants.CanIDs.LEFT_ELEVATOR_TALON);
    if (!Utils.isSimulation()) motorConfig.withHardwareLimitSwitch(limitSwitch);
    masterMotor.getConfigurator().apply(motorConfig);

    followerMotor = new TalonFX(Constants.CanIDs.RIGHT_ELEVATOR_TALON);
    followerMotor.getConfigurator().apply(motorConfig);

    // Motion Magic / Voltage Requests
    positionRequest =
        new DynamicMotionMagicVoltage(
            0,
            Constants.ElevatorConstants.CRUISE_VELOCITY,
            Constants.ElevatorConstants.MM_UPWARD_ACCELERATION,
            Constants.ElevatorConstants.MM_JERK);

    brakeRequest =
        new DynamicMotionMagicVoltage(
            0,
            Constants.ElevatorConstants.CRUISE_VELOCITY,
            Constants.ElevatorConstants.MM_BRAKE_ACCELERATION,
            0);

    sysIdRequest = new VoltageOut(0);

    masterMotor.setControl(brakeRequest);
    followerMotor.setControl(new Follower(masterMotor.getDeviceID(), false));

    // Simulation support
    addTalonSimModel(
        masterMotor, DCMotor.getKrakenX60Foc(2), Constants.ElevatorConstants.GEAR_RATIO);

    masterMotor.setPosition(Constants.ElevatorConstants.DEFAULT_POSITION);

    // SysID routine
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // rampRate
                Volts.of(12), // stepVoltage
                Seconds.of(10.0), // timeoutSeconds
                state ->
                    SignalLogger.writeString("SysIdElevator_State", state.toString()) // recordState
                ),
            new SysIdRoutine.Mechanism(
                output -> masterMotor.setControl(sysIdRequest.withOutput(output)),
                log -> {}, // optional log consumer
                this // subsystem reference
                ));
  }

  @Override
  public void periodic() {
    super.periodic();

    double latencyCompPos =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            masterMotor.getPosition(), masterMotor.getVelocity());

    atSetpoint =
        Math.abs(latencyCompPos - positionRequest.getPositionMeasure().in(Rotations))
            <= Constants.ElevatorConstants.SETPOINT_TOLERANCE;
    getNetworkTable().getEntry("At Setpoint").setBoolean(atSetpoint);
  }

  @Override
  public boolean setDesiredState(SubsystemState desiredState) {
    if (!super.setDesiredState(desiredState)) return false;

    Double position = desiredState.position;
    if (position == null) {
      brakeRequest.Position = masterMotor.getPosition().getValueAsDouble();
      masterMotor.setControl(brakeRequest);
    } else {
      if (masterMotor.getPosition().getValue().in(Rotations) < position) {
        positionRequest.Acceleration = Constants.ElevatorConstants.MM_UPWARD_ACCELERATION;
      } else {
        positionRequest.Acceleration = Constants.ElevatorConstants.MM_DOWNWARD_ACCELERATION;
      }
      positionRequest.Position = position;
      masterMotor.setControl(positionRequest);
    }

    return true;
  }

  public boolean isAtSetpoint() {
    if (getCurrentState() == SubsystemState.IDLE) return false;
    double latencyCompPos =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            masterMotor.getPosition(), masterMotor.getVelocity());
    atSetpoint =
        Math.abs(latencyCompPos - positionRequest.getPositionMeasure().in(Rotations))
            <= Constants.ElevatorConstants.SETPOINT_TOLERANCE;
    return atSetpoint;
  }

  public Command stop() {
    return new InstantCommand(() -> masterMotor.setControl(brakeRequest), this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction).andThen(stop());
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction).andThen(stop());
  }

  public double getHeight() {
    return (masterMotor.getPosition().getValueAsDouble() / Constants.ElevatorConstants.GEAR_RATIO)
        * (2 * Math.PI * 0.508);
  }

  public Pose3d[] getComponentPoses() {
    double pos = masterMotor.getPosition().getValueAsDouble();
    return new Pose3d[] {
      new Pose3d(0, 0, pos * (0.6985 / 6.096924), new Rotation3d()),
      new Pose3d(0, 0, pos * 2 * (0.6985 / 6.096924), new Rotation3d())
    };
  }

  public Pose3d[] getTargetPoses() {
    double reference = masterMotor.getClosedLoopReference().getValueAsDouble();
    return new Pose3d[] {
      new Pose3d(0, 0, reference * (0.6985 / 6.096924), new Rotation3d()),
      new Pose3d(0, 0, reference * 2 * (0.6985 / 6.096924), new Rotation3d())
    };
  }
}
