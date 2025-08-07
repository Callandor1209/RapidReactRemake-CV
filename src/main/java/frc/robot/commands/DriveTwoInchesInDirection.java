// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveTwoInchesInDirection extends Command {
  char _direction;
  private final double _twoInchesInMeters = 0.0508;
      private Timer _timer = new Timer();
  Pose2d _targetPose;
     private PIDController _drivePID = new PIDController(12, 0, 0);
  /** Creates a new DriveTwoInchesInDirection. */
  public DriveTwoInchesInDirection( char direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.DRIVETRAIN_SUBSYSTEM);
    _direction = direction;
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer.start();
    switch (_direction) {
        case 'F':
            _targetPose = Robot.DRIVETRAIN_SUBSYSTEM.getTargetPoseFromRobotRelativeChange(_twoInchesInMeters, 0);
            break;
        case 'B':
            _targetPose = Robot.DRIVETRAIN_SUBSYSTEM.getTargetPoseFromRobotRelativeChange(-_twoInchesInMeters, 0);
            break;
        case 'L':
            _targetPose = Robot.DRIVETRAIN_SUBSYSTEM.getTargetPoseFromRobotRelativeChange(0, _twoInchesInMeters);
            break;
        case 'R':
            _targetPose = Robot.DRIVETRAIN_SUBSYSTEM.getTargetPoseFromRobotRelativeChange(0, -_twoInchesInMeters);
            break;
            default:
            throw new IllegalArgumentException("Invalid direction");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
          double[] speeds = Robot.DRIVETRAIN_SUBSYSTEM.calculateSpeeds(_targetPose, _drivePID);
      ChassisSpeeds speed = new ChassisSpeeds(speeds[0], speeds[1], 0);
      Robot.DRIVETRAIN_SUBSYSTEM.driveAutoWithFeedforward(ChassisSpeeds.fromFieldRelativeSpeeds(speed, Robot.DRIVETRAIN_SUBSYSTEM.getGyroscopeRotation()), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
       Transform2d pose = Robot.DRIVETRAIN_SUBSYSTEM.getPose().minus(_targetPose);
        if ( (_timer.get() > 1 ||Math.abs(pose.getX()) < 0.00254 && Math.abs(pose.getY()) < 0.00254)) { // 1/10 of an inch
            System.out.println("Final drive two inches pose: " + Robot.DRIVETRAIN_SUBSYSTEM.getPose());
            return true;
        }
        return false;
    }

  
}
