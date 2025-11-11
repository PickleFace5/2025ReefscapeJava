package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/** Subsystem class for handling subsystem state transitions and motor simulation models. */
public abstract class StateSubsystem<S extends Enum<S>> extends SubsystemBase {

  // VERY IMPORTANT: This subsystem architecture does not translate well to Java.
  // It would be better off just not having a shared class like this.
  // See the dev branch for a much better way that the subsystems were done w/ AdvantageKit and IOs

  /** Represents the possible subsystem states. */
  public enum SubsystemState {
    OFF
  }

  private boolean frozen;
  private S subsystemState;
  private final S startingState;

  private final NetworkTable networkTable;
  private final List<SimModel> simModels = new ArrayList<>();

  private final StringPublisher statePub;
  private final BooleanPublisher frozenPub;

  /**
   * Initializes all subsystem logging and sets the default state.
   *
   * @param name the name of the subsystem
   * @param startingState the starting state of the subsystem
   */
  public StateSubsystem(String name, S startingState) {
    setName(capitalize(name));
    this.startingState = startingState;
    this.frozen = false;
    this.subsystemState = null;

    // Create NT folder for organization
    this.networkTable = NetworkTableInstance.getDefault().getTable(this.getName());
    this.statePub = networkTable.getStringTopic("Current State").publish();
    this.statePub.set("INITIALIZING");

    this.frozenPub = networkTable.getBooleanTopic("Frozen").publish();
    this.frozenPub.set(frozen);
  }

  /**
   * Sets the desired state of the subsystem. It’s recommended to override this function to update
   * objects such as control requests.
   *
   * @param desiredState the desired state
   * @return true if the state changed, false otherwise
   */
  public boolean setDesiredState(S desiredState) {
    if (subsystemState == desiredState || DriverStation.isTest() || isFrozen()) {
      return false;
    }
    subsystemState = desiredState;
    statePub.set(getStateName());
    return true;
  }

  @Override
  public void periodic() {
    // Ensure initial state is set once
    if (subsystemState == null) {
      setDesiredState(startingState);
    }

    // Skip simulation updates if not sim
    if (!Utils.isSimulation()) {
      return;
    }

    // Update simulated devices
    for (SimModel model : simModels) {
      TalonFX talon = model.talon;
      DCMotorSim sim = model.sim;

      var simState = talon.getSimState();
      simState.setSupplyVoltage(RobotController.getBatteryVoltage());

      sim.setInputVoltage(simState.getMotorVoltage());
      sim.update(0.02);

      simState.setRawRotorPosition(
          Units.radiansToRotations(sim.getAngularPosition().in(Radians)) * model.gearing);
      simState.setRotorVelocity(
          Units.radiansToRotations(sim.getAngularVelocity().in(RadiansPerSecond)) * model.gearing);
      simState.setRotorAcceleration(
          Units.radiansToRotations(sim.getAngularAcceleration().in(RadiansPerSecondPerSecond))
              * model.gearing);
    }
  }

  /** Prevents new state changes. */
  public void freeze() {
    frozen = true;
    frozenPub.set(true);
  }

  /** Allows state changes again. */
  public void unfreeze() {
    frozen = false;
    frozenPub.set(false);
  }

  public boolean isFrozen() {
    return frozen;
  }

  public S getCurrentState() {
    return subsystemState;
  }

  /** Returns the name of the current state. */
  public String getStateName() {
    return subsystemState == null ? "None" : capitalize(subsystemState.name().replace("_", " "));
  }

  public NetworkTable getNetworkTable() {
    return networkTable;
  }

  /**
   * Adds a TalonFX simulation model that updates periodically during simulation.
   *
   * @param talon The TalonFX to simulate
   * @param motor The motor type on the physical robot
   * @param gearing The gearing from the TalonFX to the mechanism
   * @param moi The moment of inertia (default 0.001)
   */
  public void addTalonSimModel(TalonFX talon, DCMotor motor, double gearing, double moi) {
    DCMotorSim sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, gearing), motor);
    simModels.add(new SimModel(sim, talon, gearing));
  }

  /** Convenience overload with default moment of inertia. */
  public void addTalonSimModel(TalonFX talon, DCMotor motor, double gearing) {
    addTalonSimModel(talon, motor, gearing, 0.001);
  }

  /** Returns a Command that sets the desired state. */
  public Command setDesiredStateCommand(S state) {
    return new InstantCommand(() -> setDesiredState(state), this);
  }

  /** Helper class to store simulation models. */
  protected record SimModel(DCMotorSim sim, TalonFX talon, double gearing) {}

  protected List<SimModel> getSimModels() {
    return simModels;
  }

  /** Capitalizes the first letter of a string. */
  private static String capitalize(String input) {
    if (input == null || input.isEmpty()) return input;
    return input.substring(0, 1).toUpperCase() + input.substring(1).toLowerCase();
  }
}
