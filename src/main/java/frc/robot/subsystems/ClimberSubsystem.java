package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

/**
 * The ClimberSubsystem is responsible for controlling the robot's climber mechanism. In order to
 * stay in the air after being disabled, the climber mechanism utilizes a ratchet powered by a
 * servo.
 */
public class ClimberSubsystem extends StateSubsystem<ClimberSubsystem.SubsystemState> {

  public enum SubsystemState {
    STOP,
    CLIMB_IN,
    CLIMB_IN_FULL,
    CLIMB_OUT
  }

  private final Servo climbServo;
  private final TalonFX climbMotor;
  private final VoltageOut climbRequest = new VoltageOut(0);

  private final DoublePublisher servoDesiredAnglePub =
      getNetworkTable().getDoubleTopic("Servo Desired Angle").publish();

  private final java.util.Map<SubsystemState, StateConfig> stateConfigs =
      java.util.Map.of(
          SubsystemState.STOP,
              new StateConfig(0, Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE),
          SubsystemState.CLIMB_IN,
              new StateConfig(
                  Constants.ClimberConstants.CLIMB_IN_VOLTAGE,
                  Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE),
          SubsystemState.CLIMB_IN_FULL,
              new StateConfig(
                  Constants.ClimberConstants.VOLTAGE_INWARDS,
                  Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE),
          SubsystemState.CLIMB_OUT,
              new StateConfig(
                  Constants.ClimberConstants.VOLTAGE_OUTWARDS,
                  Constants.ClimberConstants.SERVO_ENGAGED_ANGLE));

  private record StateConfig(double voltage, double servoAngle) {}

  public ClimberSubsystem() {
    super("Climber", SubsystemState.STOP);

    climbServo = new Servo(Constants.ClimberConstants.SERVO_PORT);
    climbMotor = new TalonFX(Constants.CanIDs.CLIMB_TALON, "Drivetrain");

    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSlot0(Constants.ClimberConstants.GAINS)
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(Constants.ClimberConstants.GEAR_RATIO));

    climbMotor.getConfigurator().apply(motorConfig);
    climbMotor.setPosition(0);

    addTalonSimModel(climbMotor, DCMotor.getFalcon500Foc(1), Constants.ClimberConstants.GEAR_RATIO);
  }

  @Override
  public boolean setDesiredState(SubsystemState desiredState) {
    if (!super.setDesiredState(desiredState)) {
      return false;
    }

    StateConfig config =
        stateConfigs.getOrDefault(
            desiredState, new StateConfig(0, Constants.ClimberConstants.SERVO_ENGAGED_ANGLE));

    climbRequest.withOutput(config.voltage);
    climbServo.setAngle(config.servoAngle);
    servoDesiredAnglePub.set(climbServo.getAngle());
    climbMotor.setControl(climbRequest);
    return true;
  }

  public double getPosition() {
    return climbMotor.getPosition().getValue().in(Rotations) * (9.0 / 64.0);
  }

  public Pose3d getComponentPose() {
    return new Pose3d(
        0.0, 0.292100, 0.463550, new Rotation3d(Math.toRadians(getPosition()), 0.0, 0.0));
  }
}
