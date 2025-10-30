package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Constants {
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static class CanIDs {
    public static final int LEFT_ELEVATOR_TALON = 10;
    public static final int RIGHT_ELEVATOR_TALON = 11;
    public static final int INTAKE_TALON = 12;
    public static final int LEFT_PIVOT_TALON = 13;
    public static final int CLIMB_TALON = 15;
    public static final int FUNNEL_TALON = 22;

    public static final int ELEVATOR_CANDI = 20;
    public static final int PIVOT_CANCODER = 21;
    public static final int INTAKE_CANRANGE = 23;
  }

  public static class ClimberConstants {
    public static final float GEAR_RATIO = 48;
    public static final Slot0Configs GAINS =
        new Slot0Configs().withKP(1.0).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.0).withKA(0.0);

    public static final float VOLTAGE_INWARDS = 4;
    public static final float VOLTAGE_OUTWARDS = -5;
    public static final float CLIMB_IN_VOLTAGE = 4;

    public static final double CLIMB_FULL_THRESHOLD = 0.44;

    public static final int SERVO_PORT = 0;
    public static final double SERVO_ENGAGED_ANGLE = 0;
    public static final int SERVO_DISENGAGED_ANGLE = 90;
  }

  public static class ElevatorConstants {
    public static final double L1_SCORE_POSITION = 2.208;
    public static final double L2_SCORE_POSITION = 1.841;
    public static final double L3_SCORE_POSITION = 3.576;
    public static final double L4_SCORE_POSITION = 6.087158;
    public static final double L2_ALGAE_POSITION = 3.198;
    public static final double L3_ALGAE_POSITION = 5;
    public static final double NET_SCORE_POSITION = 6.052246;
    public static final double PROCESSOR_SCORE_POSITION = 0.8205;
    public static final double ELEVATOR_MAX = 6.096924;

    public static final double DEFAULT_POSITION = 0;

    public static final double CRUISE_VELOCITY = 9.5;
    public static final double MM_JERK = 6000;
    public static final double MM_UPWARD_ACCELERATION = 65;
    public static final double MM_BRAKE_ACCELERATION = 24;
    public static final double MM_DOWNWARD_ACCELERATION = 12;
    public static final double EXPO_K_V = 10;
    public static final double EXPO_K_A = 4;

    public static final double GEAR_RATIO = 31.0 / 4.0;
    public static final Slot0Configs GAINS =
        new Slot0Configs()
            .withKG(0.352)
            .withKP(40)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.062)
            .withKV(0.0)
            .withKA(0.0)
            .withGravityType(GravityTypeValue.Elevator_Static);

    public static final double SETPOINT_TOLERANCE = 0.1;
  }

  public static class PivotConstants {
    public static final double INSIDE_ELEVATOR_ANGLE = 0.279297;
    public static final double ELEVATOR_PRIORITY_ANGLE = 0.241943;
    public static final double STOW_ANGLE = 0.267334;
    public static final double GROUND_INTAKE_ANGLE = -0.054688 ;
    public static final double FUNNEL_INTAKE_ANGLE = 0.373;
    public static final double ALGAE_INTAKE_ANGLE = -0.04;
    public static final double HIGH_SCORING_ANGLE = 0.30;
    public static final double MID_SCORING_ANGLE = 0.30;
    public static final double LOW_SCORING_ANGLE = -0.065186;
    public static final double NET_SCORING_ANGLE = 0.131;
    public static final double PROCESSOR_SCORING_ANGLE = 0.001;
    public static final double CLIMBER_PRIORITY_ANGLE = 0.349365;

    public static final double MINIMUM_ANGLE = -0.091;
    public static final double MAXIMUM_ANGLE = 0.392822;

    public static final double CRUISE_VELOCITY = 3;
    public static final double MM_ACCELERATION = 3;

    public static final double GEAR_RATIO = 23.0;
    public static final Slot0Configs GAINS =
        new Slot0Configs()
            .withKG(0.518)
            .withKP(45)
            .withKI(0.0)
            .withKD(0.6343)
            .withKS(0.26)
            .withKV(0.0)
            .withKA(0.0)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final double CANCODER_DISCONTINUITY = 0.5;
    public static final double CANCODER_OFFSET = -0.168701171875;

    public static final double SETPOINT_TOLERANCE = 0.03125;
  }

  public static class IntakeConstants {
    public static final double CORAL_INTAKE_SPEED = 0.4 * 1.2 * 1.1;
    public static final double FUNNEL_INTAKE_SPEED = 0.8 * 0.75;
    public static final double CORAL_OUTPUT_SPEED = 0.6;
    public static final double L1_OUTPUT_SPEED = -0.4;

    public static final double ALGAE_HOLD = 0.125;
    public static final double ALGAE_INTAKE_SPEED = 0.75;
    public static final double ALGAE_OUTPUT_SPEED = -1;

    public static final int SUPPLY_CURRENT = 35;

    public static final int GEAR_RATIO = 4;
    public static final Slot0Configs GAINS =
        new Slot0Configs().withKP(1.0).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.0).withKA(0.0);
  }

  public static class FunnelConstants {
    public static final double CORAL_STATION_POSITION = 0.24;
    public static final double STOWED_POSITION = 0;
    public static final double GEAR_RATIO = 12;
    public static final double CRUISE_VELOCITY = 1;
    public static final double SETPOINT_TOLERANCE = 0.01;
    public static final double MM_ACCELERATION = 3.5;
    public static final Slot0Configs GAINS =
        new Slot0Configs()
            .withKP(45)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKG(0.25)
            .withGravityType(GravityTypeValue.Arm_Cosine);

    public static final int SUPPLY_CURRENT = 20;
    public static final int STATOR_CURRENT = 50;
  }

  public static class VisionConstants {
    public static final String FRONT_LEFT = "limelight-fl";
    public static final String FRONT_RIGHT = "limelight-fr";
    public static final String FRONT_CENTER = "limelight-front";
    public static final String BACK_CENTER = "limelight-back";
  }

  public static class AutoAlignConstants {
    public static final double TRANSLATION_P = 9;
    public static final double TRANSLATION_I = 0;
    public static final double TRANSLATION_D = 0.1;

    public static final double HEADING_P = 1;
    public static final double HEADING_I = 0;
    public static final double HEADING_D = 0.2;
  }
}
