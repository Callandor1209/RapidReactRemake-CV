// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoCommand extends Command {
  /** Creates a new autoCommand. */
  CommandSwerveDrivetrain driveTrainSubsystem = Robot.DRIVETRAIN_SUBSYSTEM;
  VisionSubsystem visionSubsystem = Robot.VISION_SUBSYSTEM;
  Timer timer = new Timer();
  double x;
  double y;
  boolean done = false;
  double x3;
  double y3;
  boolean done2 = false;
  public autoCommand(double x2, double y2) {
    // Use addRequirements() here to declare subsystem dependencies.
    x = x2;
    y = y2;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( 
      new Pose2d(driveTrainSubsystem.getPose().getX(),driveTrainSubsystem.getPose().getY(), new Rotation2d()),
      new Pose2d(x,y, new Rotation2d())
    );
    PathConstraints pathConstraints = new PathConstraints(3, 3, 540, 720); //this is just pulled from pathplanner defaults for now
    PathPlannerPath path = new PathPlannerPath(waypoints, pathConstraints, null, new GoalEndState(0, Rotation2d.fromDegrees(0)));
    Command followpath = AutoBuilder.followPath(path);
     followpath.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(driveTrainSubsystem.getPose().getX() - x) < 0.1 && Math.abs(driveTrainSubsystem.getPose().getY() - y) < 0.1 && !done ){
      Robot.INTAKE_SUBSYSTEM.startIntakeConveyer();
      new TurretAuto();
      timer.start();
      done = true;
      driveTrainSubsystem.noDefault = true;
      
    }
    if(timer.get() > 2){

    }
    if(done){
      System.out.println("attempting to turn");
      driveTrainSubsystem.drive(0, 0, 10);
    }
    if(done && visionSubsystem.returnXandYTranslation()[1] != 0){
      driveTrainSubsystem.drive(0, 0, 0);
      x3 = visionSubsystem.returnXandYTranslation()[0] + driveTrainSubsystem.getPose().getX();
      y3 = visionSubsystem.returnXandYTranslation()[1] + driveTrainSubsystem.getPose().getY();
      done2 = true;
    }
    System.out.println(timer.get());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new autoCommand(x3,y3).schedule();
    driveTrainSubsystem.noDefault = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done2;
  }
}
