package frc.robot.subsystems.funnel;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class FunnelSubsystem extends SubsystemBase {
  public enum State {
    UP(Constants.FunnelConstants.CORAL_STATION_POSITION),
    DOWN(Constants.FunnelConstants.STOWED_POSITION);

    public final double position;

    State(double position) {
      this.position = position;
    }
  }

  private final FunnelIO io;
  private final FunnelIOInputsAutoLogged inputs = new FunnelIOInputsAutoLogged();

  private final Alert funnelDisconnectedAlert;

  private State currentState = State.DOWN;

  public FunnelSubsystem(FunnelIO io) {
    this.io = io;
    setName("Funnel");

    funnelDisconnectedAlert = new Alert("Funnel motor disconnected.", Alert.AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Funnel", inputs);

    funnelDisconnectedAlert.set(!inputs.funnelConnected);
  }

  @AutoLogOutput(key = "Funnel/State")
  public State getState() {
    return currentState;
  }

  public void setState(State state) {
    currentState = state;
    io.setFunnelPosition(Rotation2d.fromRotations(currentState.position));
  }

  public Pose3d getComponentPose() {
    return new Pose3d(
        new Translation3d(-0.311150, 0, 0.703243),
        new Rotation3d(0, -inputs.funnelPosition.getRadians(), 0));
  }
}
