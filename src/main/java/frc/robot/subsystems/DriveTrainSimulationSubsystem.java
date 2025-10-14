// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DriveTrainSimulationSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSimulationSubsystem. */
  public DriveTrainSimulationSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    public double[][] getRobotArea(){
    Pose2d pose2d = Robot.DRIVETRAIN_SUBSYSTEM.getPose();
    double leftBoundry =  pose2d.getX() - 0.318 - 0.35;
    double rightBoundry = pose2d.getX() + 0.318 + 0.35;
    double upperBoundry = pose2d.getY() + 0.318 + 0.35;
    double lowerBoundry = pose2d.getY() - 0.318 - 0.35;
    double[] rightUpperCorner = rotatePoint(rightBoundry, upperBoundry);
    double[] leftUpperCorner = rotatePoint(leftBoundry,upperBoundry);
    double[] rightLowerCorner = rotatePoint(rightBoundry, lowerBoundry);
    double[] leftLowerCorner = rotatePoint(leftBoundry,lowerBoundry);
    double[][] doubleArray = {rightLowerCorner,rightUpperCorner,leftLowerCorner,leftUpperCorner};
    return doubleArray;

  }

    //Maths
    public double[] rotatePoint(double pointX, double pointY){
      double degrees = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getRotation().getRadians();
      double newOriginX = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getX();
      double newOriginY = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getY();
      double translatedX = pointX - newOriginX;
      double translatedY = pointY - newOriginY;
      double newpointX = translatedX * Math.cos(degrees) - translatedY * Math.sin(degrees);
      double newpointY = translatedX * Math.sin(degrees) + translatedY * Math.cos(degrees);
      pointX = newpointX + newOriginX;
      pointY = newpointY + newOriginY;
      double[] doubleArray = {pointX,pointY};
      return doubleArray; 
      }
}
