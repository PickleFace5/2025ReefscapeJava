package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  public enum State {
    HOLD(0, false),
    ALGAE_HOLD(Constants.IntakeConstants.ALGAE_HOLD, true),
    CORAL_INTAKE(Constants.IntakeConstants.CORAL_INTAKE_SPEED, false),
    FUNNEL_INTAKE(Constants.IntakeConstants.FUNNEL_INTAKE_SPEED, false),
    CORAL_OUTPUT(Constants.IntakeConstants.CORAL_OUTPUT_SPEED, true),
    ALGAE_INTAKE(Constants.IntakeConstants.ALGAE_INTAKE_SPEED, false),
    ALGAE_OUTPUT(Constants.IntakeConstants.ALGAE_OUTPUT_SPEED, true),
    L1_OUTPUT(Constants.IntakeConstants.L1_OUTPUT_SPEED, true);

    public final double output;
    public final boolean ignoreLimits;

    State(double output, boolean ignoreLimits) {
      this.output = output;
      this.ignoreLimits = ignoreLimits;
    }
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert intakeDisconnectedAlert;

  private State currentState = State.HOLD;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
    setName("Intake");

    intakeDisconnectedAlert = new Alert("Intake motor disconnected.", Alert.AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    intakeDisconnectedAlert.set(!inputs.intakeConnected);
  }

  @AutoLogOutput(key = "Intake/State")
  public State getState() {
    return currentState;
  }

  public void setState(State state) {
    currentState = state;
    boolean ignoringLimits = state.ignoreLimits;
    Logger.recordOutput("Intake/Ignoring Limit Switch", ignoringLimits);
    io.setIntakeOpenLoop(currentState.output, ignoringLimits);
  }

  public Command setDesiredStateCommand(State state) {
    return new InstantCommand(() -> setState(state), this);
  }

  public boolean hasCoral() {
    return inputs.sensorSensed;
  }
}
