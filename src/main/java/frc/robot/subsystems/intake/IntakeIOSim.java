package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor INTAKE_GEARBOX = DCMotor.getFalcon500Foc(1);
  private final DCMotorSim intakeSim;

  private double intakeAppliedVolts = 0.0;

  public IntakeIOSim() {
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                INTAKE_GEARBOX, 0.01, Constants.IntakeConstants.GEAR_RATIO),
            INTAKE_GEARBOX);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.setInputVoltage(MathUtil.clamp(intakeAppliedVolts, -12, 12));
    intakeSim.update(0.02);

    inputs.intakeConnected = true;
    inputs.intakeVelocityRadPerSec = intakeSim.getAngularVelocityRadPerSec();
    inputs.intakeAppliedVolts = intakeAppliedVolts;
    inputs.intakeCurrentAmps = Math.abs(intakeSim.getCurrentDrawAmps());
  }

  @Override
  public void setIntakeOpenLoop(double output, boolean ignoreLimits) {
    intakeAppliedVolts = output;
  }
}
