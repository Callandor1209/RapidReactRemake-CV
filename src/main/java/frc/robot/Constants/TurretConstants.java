package frc.robot.Constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.Robot;

public class TurretConstants {
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
    return slot0Config;
  }

  public static final double MOTION_MAGIC_ACCELERATION = 10;
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 5;
  public static final double MOTOR_POWER_FEEDFORWARD_AT_HORIZONTAL = 0;
}
