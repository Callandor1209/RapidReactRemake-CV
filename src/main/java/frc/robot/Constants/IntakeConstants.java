package frc.robot.Constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.Robot;

public class IntakeConstants {
    public static Slot0Configs getSlot0Configs() {
    var slot0Config = new Slot0Configs();
    slot0Config.kP = 60;
    slot0Config.kD = 1;
    slot0Config.kG = .35;
    if (Robot.isSimulation()) {
      slot0Config.kP = 10;
      slot0Config.kG = 0;
      slot0Config.kD = 0;
    }

    slot0Config.GravityType = GravityTypeValue.Arm_Cosine;
    return slot0Config;
  }

  public static final double MOTION_MAGIC_ACCELERATION = 10;
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 5;
  public static final double MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL = 0;
  public static final double REVERSE_SOFT_LIMIT_POSITION = -0.135;
  public static final double FORWARD_SOFT_LIMIT_POSITION = 0.53;




  public static enum INTAKE_POSITIONS {
    INTAKE_POSITION_UP(-2, -2), // needs updating
    INTAKE_POSITION_DOWN(0,0);
    private double position;
    private double borderPosition;

    public double getPosition() {
      return position;
    }

    public double getBorderPosition() {
      return borderPosition;
    }

    INTAKE_POSITIONS(double position, double borderPosition) {
      this.position = position;
      this.borderPosition = borderPosition;
    }
  };
}
