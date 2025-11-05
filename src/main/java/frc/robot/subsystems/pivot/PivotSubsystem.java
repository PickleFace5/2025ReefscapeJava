package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {
  public enum State {
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

    public final Rotation2d position;

    State(double position) {
      this.position = Rotation2d.fromRotations(position);
    }
  }

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private final Alert pivotDisconnectedAlert;
  private final Alert encoderDisconnectedAlert;

  private State currentState = State.STOW;

  private final SysIdRoutine sysIdRoutine;

  private final Debouncer atSetpointDebounce;

  public PivotSubsystem(PivotIO io) {
    this.io = io;
    setName("Pivot");

    pivotDisconnectedAlert = new Alert("Pivot motor disconnected.", Alert.AlertType.kError);
    encoderDisconnectedAlert = new Alert("Pivot encoder disconnected.", Alert.AlertType.kError);

    atSetpointDebounce = new Debouncer(0.1, Debouncer.DebounceType.kRising);

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
                output -> io.setOpenLoop(output.in(Volts)),
                log -> {}, // optional log consumer
                this // subsystem reference
                ));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    pivotDisconnectedAlert.set(!inputs.pivotConnected);
    encoderDisconnectedAlert.set(!inputs.pivotEncoderConnected);
  }

  @AutoLogOutput(key = "Pivot/At Setpoint")
  public boolean isAtSetpoint() {
    return atSetpointDebounce.calculate(
        Math.abs(inputs.pivotAbsolutePosition.minus(currentState.position).getRotations())
            <= Constants.PivotConstants.SETPOINT_TOLERANCE);
  }

  public void setState(State state) {
    currentState = state;
    io.setPosition(currentState.position);
  }

  @AutoLogOutput(key = "Pivot/State")
  public State getState() {
    return currentState;
  }

  @AutoLogOutput(key = "Pivot/Outside Elevator")
  public boolean isOutsideElevator() {
    return !(inputs.pivotPosition.getRotations() >= Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE);
  }

  public Command stop() {
    return runOnce(() -> io.setOpenLoop(0.0));
  }

  public void sysIdQuasistatic(SysIdRoutine.Direction direction) {
    sysIdRoutine.quasistatic(direction).andThen(this.stop());
  }

  public void sysIdDynamic(SysIdRoutine.Direction direction) {
    sysIdRoutine.dynamic(direction).andThen(this.stop());
  }

  public Pose3d getComponentPose(Pose3d carriagePose) {
    return new Pose3d(
        new Translation3d(0.32385, 0, carriagePose.getZ() + 0.2667),
        new Rotation3d(0, -inputs.pivotPosition.getRadians(), 0));
  }
}
