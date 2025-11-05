package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public boolean intakeConnected = false;
    public double intakeVelocityRadPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;

    public boolean sensorSensed = false;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void setIntakeOpenLoop(double output, boolean ignoreLimits) {}
}
