package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim climbSim;
  private double servoAngle = 0.0;
  private double appliedVoltage;

  public ClimberIOSim() {
    climbSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500Foc(1), 0.01, Constants.ClimberConstants.GEAR_RATIO),
            DCMotor.getFalcon500Foc(1));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climbSim.update(0.02);

    inputs.appliedVolts = appliedVoltage;
    inputs.velocityRadPerSec = climbSim.getAngularVelocityRadPerSec();
    inputs.positionRad = climbSim.getAngularPositionRad();
    inputs.currentAmps = Math.abs(climbSim.getCurrentDrawAmps());
    inputs.servoTargetAngleDeg = servoAngle;

    inputs.motorConnected = true;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVoltage = volts;
    climbSim.setInputVoltage(volts);
  }

  @Override
  public void setServoAngle(double angleDeg) {
    servoAngle = angleDeg;
  }
}
