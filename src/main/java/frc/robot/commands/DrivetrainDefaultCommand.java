// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.Constants;
import frc.robot.util.AngularPositionHolder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DrivetrainDefaultCommand extends Command {
  public static double x2;
  public static double y2;

  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.DRIVETRAIN_SUBSYSTEM);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d robotRotation = Robot.DRIVETRAIN_SUBSYSTEM.getPoseMA().getRotation();
    double targetRotation = robotRotation.getRadians();
    double controllerDirection = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 1 : -1;
    double x = Robot.m_driverController.getRawAxis(1) * controllerDirection;
    double y = Robot.m_driverController.getRawAxis(0) * controllerDirection;
    // playstation controller
    double r = findR(Robot.m_driverController.getRawAxis(5), Robot.m_driverController.getRawAxis(2));
    // xbox controller
     //double r = findR(Robot.m_driverController.getRawAxis(5) ,Robot.m_driverController.getRawAxis(4));
    x = squareAndKeepSign(x);
    y = squareAndKeepSign(y);
    r = squareAndKeepSign(r);
    x = applyDeadband(x);
    y = applyDeadband(y);
    r = applyRotationalDeadband(r);
    double maxVelocity = Robot.DRIVETRAIN_SUBSYSTEM.getMaxVelocity();
    x = x * maxVelocity;
    y = y * maxVelocity;
    r = r * Robot.DRIVETRAIN_SUBSYSTEM.getMaxRotationalVelocity();

    if (Robot.m_driverController.L1().getAsBoolean()|| Robot.M_XBOX_CONTROLLER.leftBumper().getAsBoolean()) {
      x = x * Constants.CUT_POWER;
      y = y * Constants.CUT_POWER;
      r = r * Constants.CUT_POWER;

    }

    r = AngularPositionHolder.GetInstance().getAngularVelocity(r, targetRotation);
    if (Robot.isSimulation()) {
      double robotX = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getX();
      double robotY = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getY();
      if(DriverStation.getAlliance().get() == Alliance.Red){
      if (y > 0 && robotY < 0.7) {
        y = y * -0.5;
      }
      if (y < 0 && robotY > 7.6) {
        y = y * -0.5;
      }
      if (x > 0 && robotX < 0.7) {
        x = x * -0.5;
      }
      if (x < 0 && robotX > 15.9) {
        x = x * -0.5;
      }
    }
    else{
      if (-y > 0 && robotY < 0.7) {
        y = y * -0.5;
      }
      if (-y < 0 && robotY > 7.6) {
        y = y * -0.5;
      }
      if (-x > 0 && robotX < 0.7) {
        x = x * -0.5;
      }
      if (-x < 0 && robotX > 15.9) {
        x = x * -0.5;
      }
    }
      /*if ((robotY - 1.26 - 4.15 < 0.5 && robotX + 0.53 - 8.25 < 0.5 && robotY - 1.26 - 4.15 > 0
          && robotX + 0.53 - 8.25 > 0)
          || (robotY + 1.26 - 4.15 < 0.5 && robotX - 0.53 - 8.25 < 0.5 && robotY + 1.26 - 4.15 > 0
              && robotX - 0.53 - 8.25 > 0)
          || (robotY - 1.26 - 4.15 < 0.5 && robotX - 0.53 - 8.25 < 0.5 && robotY - 1.26 - 4.15 > 0
              && robotX - 0.53 - 8.25 > 0)
          || (robotY + 1.26 - 4.15 < 0.5 && robotX + 0.53 - 8.25 < 0.5 && robotY + 1.26 - 4.15 > 0
              && robotX + 0.53 - 8.25 > 0)) {
        y = y * -1;
        x = x * -1;
        
      } */
    }
    updateXandY(x, y);
    if(!Robot.DRIVETRAIN_SUBSYSTEM.noDefault){

    Robot.DRIVETRAIN_SUBSYSTEM.drive(x, y, -r);
    }
  }

  private double squareAndKeepSign(double num) {
    double sign = Math.copySign(1, num);
    return num * num * sign;
  }

  private double applyDeadband(double x) {
    if (Math.abs(x) > 0.0144) {
      double percentage = (Math.abs(x) - 0.0144) / (1 - 0.0144);
      double sign = Math.copySign(1, x);
      double power = 0.07 / Robot.DRIVETRAIN_SUBSYSTEM.getMaxVelocity() * sign
          + (1 - 0.07 / Robot.DRIVETRAIN_SUBSYSTEM.getMaxVelocity()) * percentage * sign;
      return power;
    } else {
      return 0;
    }
  }

  private double applyRotationalDeadband(double x) {
    if (Math.abs(x) > 0.0144) {
      double percentage = (Math.abs(x) - 0.0144) / (1 - 0.0144);
      double sign = Math.copySign(1, x);
      double power = 0.01 / Robot.DRIVETRAIN_SUBSYSTEM.getMaxRotationalVelocity() * sign
          + (1 - 0.01 / Robot.DRIVETRAIN_SUBSYSTEM.getMaxRotationalVelocity()) * percentage * sign;
      return power;
    } else {
      return 0;
    }
  }

  public double findR(double axis5, double axis2) {
    if (axis2 > 0) {
      if (axis2 + Math.abs(axis5) > 1) {
        return 1;
      }
      return axis2 + Math.abs(axis5);
    }
    if (axis2 - Math.abs(axis5) < -1) {
      return -1;
    }
    return axis2 + -(Math.abs(axis5));
  }

  public void updateXandY(double x3, double y3) {
    x2 = x3;
    y2 = y3;
  }

  public static double[] x2y2return() {
    double[] doublearray = { x2, y2 };
    return doublearray;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.DRIVETRAIN_SUBSYSTEM.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
