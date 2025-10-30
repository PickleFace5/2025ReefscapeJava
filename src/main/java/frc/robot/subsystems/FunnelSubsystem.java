package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class FunnelSubsystem extends StateSubsystem<FunnelSubsystem.SubsystemState> {

  public enum SubsystemState {
    UP,
    DOWN
  }

  private static final double UP_POSITION = Constants.FunnelConstants.CORAL_STATION_POSITION;
  private static final double DOWN_POSITION = Constants.FunnelConstants.STOWED_POSITION;

  private static final TalonFXConfiguration FUNNEL_CONFIG;

  static {
    FUNNEL_CONFIG = new TalonFXConfiguration();
    FUNNEL_CONFIG.Feedback.SensorToMechanismRatio = Constants.FunnelConstants.GEAR_RATIO;
    FUNNEL_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    FUNNEL_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    FUNNEL_CONFIG.Slot0.kP = Constants.FunnelConstants.GAINS.kP;
    FUNNEL_CONFIG.Slot0.kI = Constants.FunnelConstants.GAINS.kI;
    FUNNEL_CONFIG.Slot0.kD = Constants.FunnelConstants.GAINS.kD;
    FUNNEL_CONFIG.MotionMagic.MotionMagicAcceleration = Constants.FunnelConstants.MM_ACCELERATION;
    FUNNEL_CONFIG.MotionMagic.MotionMagicCruiseVelocity = Constants.FunnelConstants.CRUISE_VELOCITY;
    FUNNEL_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    FUNNEL_CONFIG.CurrentLimits.SupplyCurrentLimit = Constants.FunnelConstants.SUPPLY_CURRENT;
    FUNNEL_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 0;
    FUNNEL_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
    FUNNEL_CONFIG.CurrentLimits.StatorCurrentLimit = Constants.FunnelConstants.STATOR_CURRENT;
  }

  private final TalonFX funnelMotor;
  private final MotionMagicVoltage positionRequest;
  private final VoltageOut brakeRequest;

  public FunnelSubsystem() {
    super("Funnel", SubsystemState.DOWN);

    funnelMotor = new TalonFX(Constants.CanIDs.FUNNEL_TALON);
    funnelMotor.getConfigurator().apply(FUNNEL_CONFIG);
    funnelMotor.setPosition(0);

    addTalonSimModel(funnelMotor, DCMotor.getFalcon500Foc(1), Constants.FunnelConstants.GEAR_RATIO);

    positionRequest = new MotionMagicVoltage(0);
    brakeRequest = new VoltageOut(0);
  }

  @Override
  public boolean setDesiredState(SubsystemState desiredState) {
    if (!super.setDesiredState(desiredState)) return false;

    positionRequest.Position = (desiredState == SubsystemState.UP) ? UP_POSITION : DOWN_POSITION;
    funnelMotor.setControl(positionRequest);
    return true;
  }

  public Pose3d getComponentPose() {
    double rotationRad = -Units.rotationsToRadians(funnelMotor.getPosition().getValueAsDouble());
    return new Pose3d(new Translation3d(-0.311150, 0, 0.703243), new Rotation3d(0, rotationRad, 0));
  }

  public Pose3d getTargetPose() {
    double rotationRad = -Units.rotationsToRadians(funnelMotor.getClosedLoopReference().getValue());
    return new Pose3d(new Translation3d(-0.311150, 0, 0.703243), new Rotation3d(0, rotationRad, 0));
  }
}
