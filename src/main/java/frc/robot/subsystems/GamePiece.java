// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class GamePiece extends SubsystemBase {
  double forwardMomentum;
  double gravity = -0.98;
  double upwardsMomentum;
  double sidewaysMomentum;
  double offset = 0.005;
  Pose3d gamePose;
  String name2;

  private boolean inRobot = false;
  double poseX;
   double poseY;
   double poseZ;
   double shootPoseX;
   double shootPoseY;
   double targetX;
   double targetY;
   private boolean isRed;
   private boolean firing = false;
   private boolean inTurret = false;
   private boolean robotRed;
   private boolean kicked;
   private boolean scored = false;
   InterpolatingDoubleTreeMap upwardsMomentumMap = new InterpolatingDoubleTreeMap();



  public GamePiece(double startPoseX,double startPoseY, String name, boolean isred) {
    poseX = startPoseX;
    poseY = startPoseY;
    poseZ = 0.13;
    name2 = name;
    isRed = isred;
    gamePose = new Pose3d(poseX, poseY, poseZ, new Rotation3d());
    if(DriverStation.getAlliance().orElse(Alliance.Red) != null){
      robotRed = true;
    }
    else{
      robotRed = false;
    }
  }

  @Override
  public void periodic() {
    double robotX = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getX();
    double robotY = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getY();
    

    if(gamePieceInRobot() && !inRobot && !kicked){
      forwardMomentum = Robot.DRIVETRAIN_SUBSYSTEM.getState().Speeds.vxMetersPerSecond * 0.1;
      sidewaysMomentum = Robot.DRIVETRAIN_SUBSYSTEM.getState().Speeds.vyMetersPerSecond * 0.1;
      kicked = true;
    } 
    else{
      if(forwardMomentum > 0){
      forwardMomentum = forwardMomentum - 0.05;
      }
      if(sidewaysMomentum > 0 ){
      sidewaysMomentum = sidewaysMomentum - 0.05;
      }
      if(forwardMomentum < 0){
        forwardMomentum = forwardMomentum + 0.05;
        }
        if(sidewaysMomentum < 0 ){
        sidewaysMomentum = sidewaysMomentum + 0.05;
        }

      if(sidewaysMomentum > 0 && sidewaysMomentum < 0.05){
        sidewaysMomentum = 0;
      }
      if(forwardMomentum > 0 && forwardMomentum < 0.05){
        forwardMomentum = 0;
      }
      if(sidewaysMomentum == 0 && forwardMomentum == 0){
        kicked = false;
      }
    }

    if(!gamePieceInRobot()&& !gamePieceInIntake()&& !inRobot){
      upwardsMomentum = upwardsMomentum - 0.002;
    }
    else{
      if(!firing){
      upwardsMomentum = 0;
      }
    }


   

    if(gamePieceInIntake() && Robot.INTAKE_SUBSYSTEM.getIntakeConveyerSpeed()> 0.1 && !inRobot && isRed == robotRed && !gamePieceInRobot()){
      if(gamePose.getZ() < 0.33){
        poseZ = poseZ + 0.01;
      
      }
      if(Math.abs(poseX - robotX) > 0.01){
        double difference = poseX - robotX;
        if(difference>0){
          poseX = poseX - 0.01;
        }
        if(difference < 0){
          poseX = poseX + 0.01;
        }
      
      }
      if(Math.abs(poseY- robotY) > 0.01){
        double difference = poseY - robotY;
        if(difference>0){
          poseY = poseY - 0.01;
        }
        if(difference<0){
          poseY = poseY + 0.01;
        }

      }
      if(Math.abs(poseX) - Math.abs(robotX) < 0.01 && Math.abs(poseY)- Math.abs(robotY) < 0.01 && poseZ- 0.33 < 0.01 && !ConveyerSubsystem.upperSensor){
        inRobot = true;
        ConveyerSubsystem.upperSensor = true;
      }
    }
    if(inRobot && !firing){
      poseX = robotX;
      poseY = robotY - offset;
      if(Robot.CONVEYER_SUBSYSTEM.getConveyerSpeed() > 0.1 ){
        if(offset < 0.115){
        offset = offset + 0.01;
        }
        if(poseZ < 0.72 && offset > 0.114 ){
          poseZ = poseZ + 0.01;
        }
        if(offset > 0.114 && poseZ >= 0.71){
          ConveyerSubsystem.turretSensor = true;
          ConveyerSubsystem.upperSensor = false;
          inTurret = true;
        }

      }
      if(Robot.CONVEYER_SUBSYSTEM.getConveyerSpeed() < -0.1 && inRobot == true){
          if(poseZ > 0.13){
            poseZ = poseZ - 0.01;
          }
        else{
          inRobot = false;
          ConveyerSubsystem.turretSensor = false;
          ConveyerSubsystem.upperSensor = false;
        }
      }
      if(ConveyerSubsystem.turretSensor){
        double angle = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getRotation().getRadians();
        double offsetX = -0.115 * Math.sin(angle);
        double offsetY = 0.115 * Math.cos(angle);
        poseX = robotX + offsetX;
        poseY = robotY + offsetY;
      }
      
    }
    if(Robot.TURRET_SUBSYSTEM.getTurretShootMotorSpeed() > 0.01 && !firing && inTurret){
      firing = true;
      upwardsMomentum = 0.03;
    }

    if(firing){
      shootPoseX =  poseX -8.25; 
      shootPoseY =  poseY -4.15 ;
      System.out.println("is here");
      if(shootPoseX > 0.03){
        poseX = poseX - 0.05;
        System.out.println("is here2");
      }
      if(shootPoseX < -0.03){
        poseX = poseX + 0.05;
        System.out.println("is here3");
      }
      if(shootPoseY > 0.03){
        poseY = poseY - 0.05;
      }
      if (shootPoseY < -0.03){
        poseY = poseY + 0.05;
      }
      if(shootPoseY > -0.03 && shootPoseY < 0.03 && shootPoseX > -0.03 && shootPoseX < 0.03 ){
        firing = false;
        inTurret = false;
        inRobot = false;
        ConveyerSubsystem.turretSensor = false;
      }
        
    }
    if(Math.abs(poseX) - 8.25 < 0.1  && Math.abs(poseX)- 8.25 > 0 && Math.abs(poseY) - 4.15 < 0.1&&  Math.abs(poseY)- 4.15 > 0  && poseZ  - 2.8 < 0.3){
    upwardsMomentum = 0;
    scored = true;
    }
    poseX = poseX + forwardMomentum;
    poseY = poseY + sidewaysMomentum;
    poseZ = poseZ + upwardsMomentum;

    if(poseZ < 0.13){
      poseZ = 0.13;
    }
    if(!scored){
    gamePose = new Pose3d(poseX, poseY, poseZ, new Rotation3d());
    }
    Logger.recordOutput("Game Piece /" + name2, gamePose);
  }

  public double[][] getRobotArea(){
    Pose2d pose2d = Robot.DRIVETRAIN_SUBSYSTEM.getPose();
    double leftBoundry =  pose2d.getX() - 0.318;
    double rightBoundry = pose2d.getX() + 0.318;
    double upperBoundry = pose2d.getY() + 0.318;
    double lowerBoundry = pose2d.getY() - 0.318;
    double[] rightUpperCorner = rotatePoint(rightBoundry, upperBoundry);
    double[] leftUpperCorner = rotatePoint(leftBoundry,upperBoundry);
    double[] rightLowerCorner = rotatePoint(rightBoundry, lowerBoundry);
    double[] leftLowerCorner = rotatePoint(leftBoundry,lowerBoundry);
    double[][] doubleArray = {rightLowerCorner,rightUpperCorner,leftLowerCorner,leftUpperCorner};
    return doubleArray;

  }
  /* 
  public boolean gamePieceInRobot(){
    if(gamePose.getX() < getRobotArea()[0][0] && gamePose.getX() > getRobotArea()[2][0] && gamePose.getY() < getRobotArea()[1][1] && gamePose.getY() > getRobotArea()[0][1]){
      return true;
    }
     return false;
  }
     */
  public boolean gamePieceInRobot(){
    double[][] points = getRobotArea();
    return isPointInRobot(gamePose.getX(), gamePose.getY(), points);
}

//ai generated, need to come back and review (to replace the original gamepieceinrobot which doesn't rotate properly, 
//instead this one takes each corner and checks if the distace in between them is crossed by a ray on both the x and y axis from the gamePiece,
// if its crossed once it's inside, 0 or two times and its outside)
private boolean isPointInRobot(double poseX, double posey, double[][] corners) {
    boolean inside = false;
    int j = corners.length - 1;
    
    for (int i = 0; i < corners.length; i++) {
        double xi = corners[i][0], yi = corners[i][1];
        double xj = corners[j][0], yj = corners[j][1];
        
        if (((yi > posey) != (yj > posey)) && (poseX < (xj - xi) * (posey - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
        j = i;
    }
    return inside;
}

  public boolean gamePieceInIntake(){
    double[] numbers = rotatePoint(Robot.DRIVETRAIN_SUBSYSTEM.getPose().getX() + 0.1, Robot.DRIVETRAIN_SUBSYSTEM.getPose().getY()-0.38);
    return Math.abs(gamePose.getX() - numbers[0]) < 0.6 && Math.abs(gamePose.getY() - numbers[1]) < 0.4;

  }
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