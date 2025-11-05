package frc.robot.subsystems.elevator;

import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean elevatorConnected = false;
    public boolean limitSwitchConnected = false;
    public double positionRads = 0.0; // Prob should convert this to meters but like...
    public double velocityRadPerSec = 0.0; // Gotta get it to work first lmao
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void setPosition(Optional<Double> position) {}
}
