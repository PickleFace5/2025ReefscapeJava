package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim pivotSim;

  private boolean closedLoop = false;
  private final ProfiledPIDController controller;
  private double appliedVolts = 0.0;

  public PivotIOSim() {
    this.pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                GEARBOX, 0.0807378172, Constants.PivotConstants.GEAR_RATIO),
            GEARBOX);

    this.controller =
        new ProfiledPIDController(
            Constants.PivotConstants.GAINS.kP / (2 * Math.PI),
            Constants.PivotConstants.GAINS.kI / (2 * Math.PI),
            Constants.PivotConstants.GAINS.kD / (2 * Math.PI),
            new TrapezoidProfile.Constraints(
                Units.rotationsToRadians(Constants.PivotConstants.CRUISE_VELOCITY),
                Units.rotationsToRadians(Constants.PivotConstants.MM_ACCELERATION)));
    controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    // Update simulation
    if (closedLoop) {
      appliedVolts = controller.calculate(pivotSim.getAngularPositionRad());
    } else {
      controller.reset(pivotSim.getAngularPositionRad(), pivotSim.getAngularVelocityRadPerSec());
    }
    pivotSim.setInputVoltage(appliedVolts);
    pivotSim.update(0.02);

    // Update inputs
    inputs.pivotConnected = true;
    inputs.pivotEncoderConnected = true;
    inputs.pivotAbsolutePosition = new Rotation2d(pivotSim.getAngularPosition());
    inputs.pivotPosition = new Rotation2d(pivotSim.getAngularPosition());
    inputs.pivotVelocityRadPerSec = pivotSim.getAngularVelocityRadPerSec();
    inputs.pivotAppliedVolts = appliedVolts;
    inputs.pivotCurrentAmps = Math.abs(pivotSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output;
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    closedLoop = true;
    controller.setGoal(rotation.getRadians());
  }
}
