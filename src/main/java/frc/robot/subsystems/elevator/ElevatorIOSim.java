package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import java.util.Optional;

public class ElevatorIOSim implements ElevatorIO {
  private static final DCMotor ELEVATOR_GEARBOX = DCMotor.getKrakenX60Foc(2);
  private final DCMotorSim elevatorSim;

  private boolean closedLoop = true;

  private final ProfiledPIDController elevatorController =
      new ProfiledPIDController(
          Constants.ElevatorConstants.GAINS.kP / (2 * Math.PI),
          0,
          0,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(Constants.ElevatorConstants.CRUISE_VELOCITY),
              Units.rotationsToRadians(Constants.ElevatorConstants.MM_UPWARD_ACCELERATION)));
  private double elevatorAppliedVolts;

  public ElevatorIOSim() {
    elevatorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ELEVATOR_GEARBOX, 0.01, Constants.ElevatorConstants.GEAR_RATIO),
            ELEVATOR_GEARBOX);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      elevatorAppliedVolts = elevatorController.calculate(elevatorSim.getAngularPositionRad());
    } else {
      elevatorController.reset(
          elevatorSim.getAngularPositionRad(), elevatorSim.getAngularVelocityRadPerSec());
    }

    // Update simulation
    elevatorSim.setInputVoltage(MathUtil.clamp(elevatorAppliedVolts, -12.0, 12.0));
    elevatorSim.update(0.02);

    // Update inputs
    inputs.elevatorConnected = true;
    inputs.limitSwitchConnected = false;
    inputs.positionRads = elevatorSim.getAngularPositionRad();
    inputs.velocityRadPerSec = elevatorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = elevatorAppliedVolts;
    inputs.currentAmps = Math.abs(elevatorSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    elevatorAppliedVolts = output;
  }

  @Override
  public void setPosition(Optional<Double> position) {
    if (position.isEmpty()) {
      setOpenLoop(0);
      return;
    }
    closedLoop = true;
    double pos = Units.rotationsToRadians(position.get());
    elevatorController.setGoal(pos);
    if (pos > elevatorSim.getAngularPositionRad())
      elevatorController.setConstraints(
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(Constants.ElevatorConstants.CRUISE_VELOCITY),
              Units.rotationsToRadians(Constants.ElevatorConstants.MM_BRAKE_ACCELERATION)));
    else
      elevatorController.setConstraints(
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(Constants.ElevatorConstants.CRUISE_VELOCITY),
              Units.rotationsToRadians(Constants.ElevatorConstants.MM_DOWNWARD_ACCELERATION)));
  }
}
