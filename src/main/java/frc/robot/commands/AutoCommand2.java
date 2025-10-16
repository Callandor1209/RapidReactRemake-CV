// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCommand2 extends Command {
    CommandSwerveDrivetrain driveTrainSubsystem = Robot.DRIVETRAIN_SUBSYSTEM;
  VisionSubsystem visionSubsystem = Robot.VISION_SUBSYSTEM;
  Timer timer = new Timer();
  double x;
  double y;
  boolean done = false;
  double x3;
  double y3;
  double x4;
  double y4;
  boolean done2 = false;
  Pose2d pose;
  /** Creates a new AutoCommand2. */
  public AutoCommand2(double x, double y) {
    // Use addRequirements() here to declare subsystem dependencies.
    x4 = x;
    y4 = y;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        if(Math.abs(driveTrainSubsystem.getPose().getX() - x4) < 0.1 && Math.abs(driveTrainSubsystem.getPose().getY() - y4) < 0.1 && !done ){
      Robot.INTAKE_SUBSYSTEM.startIntakeConveyer();
      new TurretAuto().schedule();
      timer.start();

      driveTrainSubsystem.noDefault = true;
      
    }
    if(timer.get() > 2){
      done = true;
    }
    if(done){
      driveTrainSubsystem.drive(0, 0, 1);
    }
    if(done && visionSubsystem.returnXandYTranslation()[1] != 0){
      driveTrainSubsystem.drive(0, 0, 0);
      /*
      if(driveTrainSubsystem.getPose().getRotation().getRotations() < 0.25 && driveTrainSubsystem.getPose().getRotation().getRotations() > 0){
        x3 = driveTrainSubsystem.getPose().getX() + visionSubsystem.returnXandYTranslation()[0]  ;
        y3 =  driveTrainSubsystem.getPose().getY() - visionSubsystem.returnXandYTranslation()[1] ;
      }
      if(driveTrainSubsystem.getPose().getRotation().getRotations() > 0.25 && driveTrainSubsystem.getPose().getRotation().getRotations() < 0.5 ){
        x3 = driveTrainSubsystem.getPose().getX() + visionSubsystem.returnXandYTranslation()[0]  ;
        y3 =  driveTrainSubsystem.getPose().getY() + visionSubsystem.returnXandYTranslation()[1] ;
      }
      if(driveTrainSubsystem.getPose().getRotation().getRotations() > -0.5 && driveTrainSubsystem.getPose().getRotation().getRotations() < -0.25){
        x3 = driveTrainSubsystem.getPose().getX() - visionSubsystem.returnXandYTranslation()[0]  ;
        y3 =  driveTrainSubsystem.getPose().getY() + visionSubsystem.returnXandYTranslation()[1] ;
      }
      if(driveTrainSubsystem.getPose().getRotation().getRotations() > -0.25 && driveTrainSubsystem.getPose().getRotation().getRotations() < 0){
        x3 = driveTrainSubsystem.getPose().getX() - visionSubsystem.returnXandYTranslation()[0]  ;
        y3 =  driveTrainSubsystem.getPose().getY() - visionSubsystem.returnXandYTranslation()[1] ;
      }
         */
         
         pose =  driveTrainSubsystem.getTargetPoseFromRobotRelativeChange(visionSubsystem.returnXandYTranslation()[0] , visionSubsystem.returnXandYTranslation()[1]);

      done2 = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new autoCommand(pose).schedule();
    driveTrainSubsystem.noDefault = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done2;
  }
}
