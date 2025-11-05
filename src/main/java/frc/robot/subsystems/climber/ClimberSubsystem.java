package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The ClimberSubsystem is responsible for controlling the robot's climber mechanism. In order to
 * stay in the air after being disabled, the climber mechanism utilizes a ratchet powered by a
 * servo.
 */
public class ClimberSubsystem extends SubsystemBase {
  public enum State {
    STOP(0.0, false),
    CLIMB_IN(Constants.ClimberConstants.CLIMB_IN_VOLTAGE, false),
    CLIMB_IN_FULL(Constants.ClimberConstants.VOLTAGE_INWARDS, false),
    CLIMB_OUT(Constants.ClimberConstants.VOLTAGE_OUTWARDS, true);

    private final double voltage;
    private final double servoAngle;

    State(double voltage, boolean engageServo) {
      this.voltage = voltage;
      this.servoAngle =
          engageServo
              ? Constants.ClimberConstants.SERVO_ENGAGED_ANGLE
              : Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE;
    }
  }

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert;

  private State currentState = State.STOP;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
    setName("Climber");

    motorDisconnectedAlert = new Alert("Climber motor disconnected.", Alert.AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    io.setVoltage(currentState.voltage);
    io.setServoAngle(currentState.servoAngle);

    motorDisconnectedAlert.set(!inputs.motorConnected);
  }

  @AutoLogOutput(key = "Climber/State")
  public State getState() {
    return currentState;
  }

  public void setState(State newState) {
    currentState = newState;
  }

  public Command setStateCommand(State newState) {
    return runOnce(() -> setState(newState));
  }

  public double getPosition() {
    return inputs.positionRad * (9.0 / 64.0);
  }

  public Pose3d getComponentPose() {
    return new Pose3d(0.0, 0.292100, 0.463550, new Rotation3d(getPosition(), 0.0, 0.0));
  }
}
