package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.rotationsToRadians;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class PivotSubsystem extends StateSubsystem<PivotSubsystem.SubsystemState> {

  public enum SubsystemState {
    IDLE(null),
    AVOID_ELEVATOR(Constants.PivotConstants.ELEVATOR_PRIORITY_ANGLE),
    STOW(Constants.PivotConstants.STOW_ANGLE),
    GROUND_INTAKE(Constants.PivotConstants.GROUND_INTAKE_ANGLE),
    FUNNEL_INTAKE(Constants.PivotConstants.FUNNEL_INTAKE_ANGLE),
    ALGAE_INTAKE(Constants.PivotConstants.ALGAE_INTAKE_ANGLE),
    HIGH_SCORING(Constants.PivotConstants.HIGH_SCORING_ANGLE),
    L3_CORAL(Constants.PivotConstants.MID_SCORING_ANGLE),
    L2_CORAL(Constants.PivotConstants.MID_SCORING_ANGLE),
    LOW_SCORING(Constants.PivotConstants.LOW_SCORING_ANGLE),
    NET_SCORING(Constants.PivotConstants.NET_SCORING_ANGLE),
    PROCESSOR_SCORING(Constants.PivotConstants.PROCESSOR_SCORING_ANGLE),
    AVOID_CLIMBER(Constants.PivotConstants.CLIMBER_PRIORITY_ANGLE);

    public final Double position;

    SubsystemState(Double position) {
      this.position = position;
    }
  }

  private final CANcoder encoder;
  private final TalonFX masterMotor;
  private final MotionMagicVoltage positionRequest;
  private final VoltageOut brakeRequest;
  private final VoltageOut sysIdRequest;
  private final SysIdRoutine sysIdRoutine;

  private final Debouncer atSetpointDebounce;
  private boolean atSetpoint;

  private static final CANcoderConfiguration ENCODER_CONFIG = new CANcoderConfiguration();

  static {
    ENCODER_CONFIG.MagnetSensor.MagnetOffset = Constants.PivotConstants.CANCODER_OFFSET;
    ENCODER_CONFIG.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        Constants.PivotConstants.CANCODER_DISCONTINUITY;
  }

  private static final TalonFXConfiguration MASTER_CONFIG = new TalonFXConfiguration();

  static {
    MASTER_CONFIG.Feedback.RotorToSensorRatio = Constants.PivotConstants.GEAR_RATIO;
    MASTER_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    MASTER_CONFIG.Feedback.FeedbackRemoteSensorID = Constants.CanIDs.PIVOT_CANCODER;

    MASTER_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    MASTER_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    MASTER_CONFIG.Slot0.kP = Constants.PivotConstants.GAINS.kP;
    MASTER_CONFIG.Slot0.kI = Constants.PivotConstants.GAINS.kI;
    MASTER_CONFIG.Slot0.kD = Constants.PivotConstants.GAINS.kD;

    MASTER_CONFIG.MotionMagic.MotionMagicCruiseVelocity = Constants.PivotConstants.CRUISE_VELOCITY;
    MASTER_CONFIG.MotionMagic.MotionMagicAcceleration = Constants.PivotConstants.MM_ACCELERATION;
  }

  private SubsystemState currentState;

  public PivotSubsystem() {
    super("Pivot", SubsystemState.STOW);
    this.currentState = SubsystemState.STOW;

    encoder = new CANcoder(Constants.CanIDs.PIVOT_CANCODER);
    masterMotor = new TalonFX(Constants.CanIDs.LEFT_PIVOT_TALON);
    masterMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;

    encoder.getConfigurator().apply(ENCODER_CONFIG);
    masterMotor.getConfigurator().apply(MASTER_CONFIG);

    this.addTalonSimModel(
        masterMotor, DCMotor.getKrakenX60Foc(1), Constants.PivotConstants.GEAR_RATIO / 10, 0.00001);

    atSetpointDebounce = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    atSetpoint = true;

    positionRequest = new MotionMagicVoltage(0);
    brakeRequest = new VoltageOut(0);
    sysIdRequest = new VoltageOut(0);

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // rampRate
                Volts.of(12), // stepVoltage
                Seconds.of(10.0), // timeoutSeconds
                state ->
                    SignalLogger.writeString("SysIdPivot_State", state.toString()) // recordState
                ),
            new SysIdRoutine.Mechanism(
                output -> masterMotor.setControl(sysIdRequest.withOutput(output)),
                log -> {}, // optional log consumer
                this // subsystem reference
                ));

    masterMotor.setPosition(encoder.getPosition().getValue());
  }

  @Override
  public void periodic() {
    super.periodic();

    double latencyCompPos =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            masterMotor.getPosition(false), masterMotor.getVelocity(false));

    atSetpoint =
        atSetpointDebounce.calculate(
            Math.abs(latencyCompPos - positionRequest.Position)
                <= Constants.PivotConstants.SETPOINT_TOLERANCE);
    getNetworkTable().getEntry("At Setpoint").setBoolean(atSetpoint);
    getNetworkTable().getEntry("In Elevator").setBoolean(!isOutsideElevator());

    // Update CANcoder sim state if simulation
    if (Utils.isSimulation()) {
      var simState = getSimModels().get(0).sim();
      var cancoderSim = encoder.getSimState();

      cancoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      cancoderSim.setRawPosition(
          simState.getAngularPosition().in(Rotations) / Constants.PivotConstants.GEAR_RATIO);

      cancoderSim.setVelocity(
          simState.getAngularVelocity().in(RotationsPerSecond)
              / Constants.PivotConstants.GEAR_RATIO);
    }
  }

  @Override
  public boolean setDesiredState(SubsystemState desiredState) {
    if (!super.setDesiredState(desiredState)) return false;

    Double position = desiredState.position;
    if (position == null) {
      masterMotor.setControl(brakeRequest);
      return true;
    }

    positionRequest.Position = position;
    masterMotor.setControl(positionRequest);
    currentState = desiredState;
    return true;
  }

  @Override
  public SubsystemState getCurrentState() {
    return currentState;
  }

  public boolean isAtSetpoint() {
    return atSetpoint;
  }

  public boolean isOutsideElevator() {
    double pos = masterMotor.getPosition(true).getValueAsDouble();
    return !(pos >= Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE);
  }

  public double getSetpoint() {
    return positionRequest.Position;
  }

  public InstantCommand stop() {

    return new InstantCommand(() -> masterMotor.setControl(sysIdRequest.withOutput(0)));
  }

  public void sysIdQuasistatic(SysIdRoutine.Direction direction) {
    sysIdRoutine.quasistatic(direction).andThen(this.stop());
  }

  public void sysIdDynamic(SysIdRoutine.Direction direction) {
    sysIdRoutine.dynamic(direction).andThen(this.stop());
  }

  public double getPosition() {
    return encoder.getPosition().getValueAsDouble();
  }

  public Pose3d getComponentPose(Pose3d carriagePose) {
    return new Pose3d(
        new Translation3d(0.32385, 0, carriagePose.getZ() + 0.2667),
        new Rotation3d(0, -rotationsToRadians(encoder.getPosition().getValueAsDouble()), 0));
  }

  public Pose3d getTargetPose(Pose3d carriagePose) {
    return new Pose3d(
        new Translation3d(0.32385, 0, carriagePose.getZ() + 0.2667),
        new Rotation3d(0, -rotationsToRadians(masterMotor.getClosedLoopReference().getValue()), 0));
  }
}
