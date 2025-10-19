// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCommand2 extends Command {
    DrivetrainSubsystem driveTrainSubsystem = Robot.DRIVETRAIN_SUBSYSTEM;
  VisionSubsystem visionSubsystem = Robot.VISION_SUBSYSTEM;
  Timer timer;
  boolean done = false;
  boolean done2 = false;

  /** Creates a new AutoCommand2. */
  public AutoCommand2() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(visionSubsystem.returnYaw()) > 10){
      done2 = true;
    }
    if(done && timer.get() > 2){
      done2 = true;
    }
    if(visionSubsystem.returnArea() > 10 &&  !done){
      done = true;
      timer.start();
      new TurretAuto().schedule();
      Robot.INTAKE_SUBSYSTEM.startIntakeConveyer();
      return;
    }
    if(Robot.DRIVETRAIN_SUBSYSTEM.getState().Speeds.vxMetersPerSecond == 0 && !done){
    driveTrainSubsystem.driveStraight();
    }

              
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //new autoCommand(pose).schedule();
    driveTrainSubsystem.noDefault = false;
    new AutoCommand1().schedule();
    driveTrainSubsystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done2;
  }




}
  
