package frc.robot.subsystems.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FunnelIOSim implements FunnelIO {
  private static final DCMotor FUNNEL_GEARBOX = DCMotor.getFalcon500Foc(1);
  private final DCMotorSim funnelSim;

  private boolean closedLoop = true;

  private final ProfiledPIDController funnelController =
      new ProfiledPIDController(
          Constants.FunnelConstants.GAINS.kP / (2 * Math.PI),
          0,
          0,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(Constants.FunnelConstants.CRUISE_VELOCITY),
              Units.rotationsToRadians(Constants.FunnelConstants.MM_ACCELERATION)));
  private double funnelAppliedVolts = 0.0;

  public FunnelIOSim() {
    funnelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FUNNEL_GEARBOX, 0.031, Constants.FunnelConstants.GEAR_RATIO),
            FUNNEL_GEARBOX);
    funnelController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(FunnelIOInputs inputs) {
    if (closedLoop) {
      funnelAppliedVolts = funnelController.calculate(funnelSim.getAngularPositionRad());
    } else {
      funnelController.reset(
          funnelSim.getAngularPositionRad(), funnelSim.getAngularVelocityRadPerSec());
    }

    // Update simulation state
    funnelSim.setInputVoltage(MathUtil.clamp(funnelAppliedVolts, -12.0, 12.0));
    funnelSim.update(0.02);

    // Update inputs
    inputs.funnelConnected = true;
    inputs.funnelPosition = new Rotation2d(funnelSim.getAngularPositionRad());
    inputs.funnelVelocityRadPerSec = funnelSim.getAngularVelocityRadPerSec();
    inputs.funnelAppliedVolts = funnelAppliedVolts;
    inputs.funnelCurrentAmps = Math.abs(funnelSim.getCurrentDrawAmps());
  }

  @Override
  public void setFunnelPosition(Rotation2d rotation) {
    closedLoop = true;
    funnelController.setGoal(rotation.getRadians());
  }
}
