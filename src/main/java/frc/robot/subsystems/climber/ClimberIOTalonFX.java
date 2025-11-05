package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX climbMotor;
  private final Servo climbServo;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public ClimberIOTalonFX() {
    climbMotor = new TalonFX(Constants.CanIDs.CLIMB_TALON, "Drivetrain");
    climbServo = new Servo(Constants.ClimberConstants.SERVO_PORT);

    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSlot0(Constants.ClimberConstants.GAINS)
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(Constants.ClimberConstants.GEAR_RATIO));
    climbMotor.getConfigurator().apply(motorConfig);
    climbMotor.setPosition(0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.motorConnected = climbMotor.isConnected();
    inputs.positionRad = climbMotor.getPosition().getValue().in(Units.Radians) * (9.0 / 64.0);
    inputs.velocityRadPerSec = climbMotor.getVelocity().getValue().in(Units.RadiansPerSecond);
    inputs.appliedVolts = climbMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = climbMotor.getSupplyCurrent().getValueAsDouble();
    inputs.servoTargetAngleDeg = climbServo.getAngle();
  }

  @Override
  public void setVoltage(double volts) {
    voltageRequest.withOutput(volts);
    climbMotor.setControl(voltageRequest);
  }

  @Override
  public void setServoAngle(double angleDeg) {
    climbServo.setAngle(angleDeg);
  }
}
