package frc.robot.subsystems.funnel;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  class FunnelIOInputs {
    public boolean funnelConnected = false;
    public Rotation2d funnelPosition = Rotation2d.kZero;
    public double funnelVelocityRadPerSec = 0.0;
    public double funnelAppliedVolts = 0.0;
    public double funnelCurrentAmps = 0.0;
  }

  default void updateInputs(FunnelIOInputs inputs) {}

  default void setFunnelPosition(Rotation2d rotation) {}
}
