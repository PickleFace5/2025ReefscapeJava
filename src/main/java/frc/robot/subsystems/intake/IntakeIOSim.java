package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor INTAKE_GEARBOX = DCMotor.getFalcon500Foc(1);
  private final DCMotorSim intakeSim;

  private double intakeAppliedVolts = 0.0;

  private final IntakeSimulation intakeSimulation;

  public IntakeIOSim(AbstractDriveTrainSimulation drivetrain) {
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Coral", // No Algae (maybe later)
            drivetrain,
            Meters.of(0.327025), // Width
            Meters.of(0.285604), // Depth away from bumpers
            IntakeSimulation.IntakeSide
                .FRONT, // Yes the intake is in the front??? (sorry its like 2am I find this funny)
            1);
    intakeSimulation.setGamePiecesCount(1);

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

    inputs.sensorSensed = intakeSimulation.getGamePiecesAmount() != 0;
  }

  @Override
  public void setIntakeOpenLoop(double output, boolean ignoreLimits) {
    intakeAppliedVolts = output;
    if (output > 0 && !ignoreLimits) {
      intakeSimulation.startIntake();
    } else if (output == 0) {
      intakeSimulation.stopIntake();
    } else {
      intakeSimulation.stopIntake();
      if (intakeSimulation.getGamePiecesAmount() > 0) {
        Superstructure.attemptCoralScore();
        intakeSimulation.setGamePiecesCount(0);
      }
    }
  }
}
