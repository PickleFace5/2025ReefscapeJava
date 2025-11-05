package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  class ClimberIOInputs {
    public boolean motorConnected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;

    public double servoTargetAngleDeg = 0.0;
  }

  /** Update all loggable inputs */
  default void updateInputs(ClimberIOInputs inputs) {}

  /** Set the motor voltage */
  default void setVoltage(double volts) {}

  /** Set the servo angle */
  default void setServoAngle(double angleDeg) {}
}
