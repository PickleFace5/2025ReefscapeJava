package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  class PivotIOInputs {
    public boolean pivotConnected = false;
    public boolean pivotEncoderConnected = false;
    public Rotation2d pivotAbsolutePosition = Rotation2d.kZero;
    public Rotation2d pivotPosition = Rotation2d.kZero;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void setPosition(Rotation2d rotation) {}
}
