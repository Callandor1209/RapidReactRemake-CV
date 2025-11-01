// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCommand1 extends Command {
  /** Creates a new autoCommand. */
  boolean robotRed;
  VisionSubsystem visionSubsystem = Robot.VISION_SUBSYSTEM;
  DrivetrainSubsystem driveTrainSubsystem = Robot.DRIVETRAIN_SUBSYSTEM;
  private Timer timer;
  double x = 0;
  double y = 0;
  Command followPath;

  public AutoCommand1() {
    // Use addRequirements() here to declare subsystem dependencies.


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      if(DriverStation.getAlliance().get() == Alliance.Red){
    robotRed = true;
  }
  else{
    robotRed = false;
  }
    driveTrainSubsystem.noDefault = true;
    driveTrainSubsystem.drive(0, 0, 1);
timer = new Timer();
timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = Math.random() * 20;
    y = Math.random() * 10;
    if(y > 8 ){
     y = 2;
    }
    if(x >  16){
     x = 2;
    }
    if(timer.get() > 6){
      Robot.ARRAY_CLASS.reverseValue = Robot.ARRAY_CLASS.reverseValue * -1;
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( 
      new Pose2d(driveTrainSubsystem.getPose().getX(),driveTrainSubsystem.getPose().getY(), new Rotation2d()),
      new Pose2d(x,y,new Rotation2d())
    );
    PathConstraints pathConstraints = new PathConstraints(3, 3, 540, 720); //this is just pulled from pathplanner defaults for now
    PathPlannerPath path = new PathPlannerPath(waypoints, pathConstraints, null, new GoalEndState(0,new Rotation2d()));
     followPath = AutoBuilder.followPath(path);
     followPath.schedule();

     timer.restart();
    }
    if(Robot.VISION_SUBSYSTEM.returnYaw() > 10 * Robot.ARRAY_CLASS.reverseValue&& Robot.VISION_SUBSYSTEM.returnYaw() < 50 + 50 * Robot.ARRAY_CLASS.reverseValue && isBallAllianceColor() && Robot.VISION_SUBSYSTEM.returnArea() >= 0.25){
      driveTrainSubsystem.drive(0, 0, -1 * Robot.ARRAY_CLASS.reverseValue);
      if(followPath != null){
      followPath.cancel();
      }
    }
    else{
      driveTrainSubsystem.drive(0, 0, 1 * Robot.ARRAY_CLASS.reverseValue);
    }
    System.out.println("In Auto Command 1");
      


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.drive(0, 0, 0);
    new AutoCommand2().schedule();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Robot.VISION_SUBSYSTEM.returnYaw()) < 10 && Robot.VISION_SUBSYSTEM.returnYaw() != 0 && isBallAllianceColor();
  }

  private boolean isBallAllianceColor(){
    if(Robot.isSimulation()){
      if(robotRed){
        return visionSubsystem.returnTagId() >= 100;
      }
      else{

        return visionSubsystem.returnTagId() < 100;
      }
    }
    return false;
  }
}
