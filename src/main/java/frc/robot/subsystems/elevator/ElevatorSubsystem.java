package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

  public enum State {
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

    State(Double position) {
      this.position = position;
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert limitSwitchAlert;
  private final Alert limitSwitchDisconnectedAlert;
  private final Alert motorDisconnectedAlert;

  private State currentState = State.DEFAULT;

  private SysIdRoutine sysIdRoutine;

  private boolean atSetpoint = true;
  private final Debouncer atSetpointDebounce;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    setName("Elevator");

    limitSwitchAlert = new Alert("Elevator limit switch Activated.", Alert.AlertType.kWarning);
    limitSwitchDisconnectedAlert =
        new Alert("Elevator limit switch disconnected.", Alert.AlertType.kError);
    motorDisconnectedAlert = new Alert("Elevator motor(s) disconnected.", Alert.AlertType.kError);

    atSetpointDebounce = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

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
                output -> io.setOpenLoop(output.in(Volts)),
                log -> {}, // optional log consumer
                this // subsystem reference
                ));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    motorDisconnectedAlert.set(!inputs.elevatorConnected);
    limitSwitchDisconnectedAlert.set(!inputs.limitSwitchConnected);
  }

  public void setState(State state) {
    currentState = state;
    io.setPosition(Optional.ofNullable(currentState.position));
  }

  @AutoLogOutput(key = "Elevator/State")
  public State getState() {
    return currentState;
  }

  @AutoLogOutput(key = "Elevator/At Setpoint")
  public boolean isAtSetpoint() {
    if (getState() == State.IDLE) return false;
    atSetpoint =
        Math.abs(Units.radiansToRotations(inputs.positionRads) - currentState.position)
            <= Constants.ElevatorConstants.SETPOINT_TOLERANCE;
    return atSetpoint;
  }

  public Command stop() {
    return new InstantCommand(() -> io.setOpenLoop(0), this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction).andThen(stop());
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction).andThen(stop());
  }

  public double getHeight() {
    return (Units.radiansToRotations(inputs.positionRads) / Constants.ElevatorConstants.GEAR_RATIO)
        * (2 * Math.PI * 0.508);
  }

  public Pose3d[] getComponentPoses() {
    double pos = Units.radiansToRotations(inputs.positionRads);
    return new Pose3d[] {
      new Pose3d(0, 0, pos * (0.6985 / 6.096924), new Rotation3d()),
      new Pose3d(0, 0, pos * 2 * (0.6985 / 6.096924), new Rotation3d())
    };
  }
}
