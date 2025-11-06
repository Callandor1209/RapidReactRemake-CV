// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Constants;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.subsystems.ConveyerSubsystem;

public class GamePiece extends SubsystemBase {
  double forwardMomentum;
  double gravity = -0.98;
  double upwardsMomentum;
  double sidewaysMomentum;
  double offset = 0.50;
  Pose3d gamePose;
  String name2;

  private boolean inRobot = false;
  double poseX = 0;
   double poseY = 0;
   double poseZ = 0;
   double targetX;
   double targetY;
   public boolean isRed;
   private boolean firing = false;
   private boolean inTurret = false;
   private boolean scored = false;
   private boolean isNew = true;
   double launchSpeedX;
   double launchSpeedY;
   double robotX;
   double robotY;
   double launchAngle;
   int aprilTagNumber;
   InterpolatingDoubleTreeMap upwardsMomentumMap = new InterpolatingDoubleTreeMap();
   InterpolatingDoubleTreeMap conveyerMapZ = new InterpolatingDoubleTreeMap();
   InterpolatingDoubleTreeMap conveyerMapY = new InterpolatingDoubleTreeMap();



  public GamePiece(double startPoseX,double startPoseY, double startPoseZ, String name, boolean isred, int aprilTagNumber2) {
    poseX = startPoseX;
    poseY = startPoseY;
    poseZ = startPoseZ;
    name2 = name;
    isRed = isred;
    gamePose = new Pose3d(poseX, poseY, poseZ, new Rotation3d());
    addThingsToTreeMap();
    aprilTagNumber = aprilTagNumber2;

  }

  @Override
  public void periodic() {
    if(scored){
      poseX = 100;
      poseY = 100;
      return;
    }
    robotX = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getX();
     robotY = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getY();
    //if the game pieces coordinates are in the robot but it has not been picked up by the robot then move it
    if(gamePieceInRobot() && !inRobot){
      runIntoGamePiece();
    } 
    else{
      slowGamePieceMovement();
      //Slow down the movement of the game piece so that it doesen't continue on for infinity
    }
    //GRAVITY!!! 
    if(isNotTouchingRobot()){
      upwardsMomentum = upwardsMomentum - 0.002;
      if(poseZ == 0.13){
        upwardsMomentum = upwardsMomentum * -0.2;
        launchSpeedX = launchSpeedX * 0.5;
      }
    }
    else{
      if(!firing){
      upwardsMomentum = 0;
      }
    }
  //if the robot is around the intake  but not already in the robot and the intake is running then pick up the game piece. 
  // oh and it has to be the same color as the robot as well
    if(isInRunningIntakeAndAbleToPickUp()){
      intakeCommand();
    }
    //used to update the values of the game piece if it is in the robot to carry it with the robot
    if(inRobot && !firing){
      moveWithRobot();
    }
    //if the game piece is in turret and the shooter motor is running
    if(ableToFire()){
      firing = true;
      upwardsMomentum = 0.03;
      ConveyerSubsystem.turretSensor = false;
    }
    //if the game piece has been fired then move it towards the center 
    if(firing){
      moveTowardsCenter(); 
    }
    //if at edge, go the other way
    if(robotOverYEdge()){
      sidewaysMomentum = sidewaysMomentum * -0.5;
    }
    if( poseX < 0.7 || poseX > 15.9){
      forwardMomentum = forwardMomentum * -0.5;
    }
    poseX = poseX + forwardMomentum;
    poseY = poseY + sidewaysMomentum;
    poseZ = poseZ + upwardsMomentum;
    // 0.13 is the value where the game piece is not clipping through the floor
    if(poseZ < 0.13){
      poseZ = 0.13;
    }
    //update the pose of the game piece
    gamePose = new Pose3d(poseX, poseY, poseZ, new Rotation3d(0,0,0));
    System.out.println(Robot.TURRET_SUBSYSTEM.getPosition());

  }


  public boolean gamePieceInRobot(){
    double[][] points = Robot.DRIVE_TRAIN_SIMULATION_SUBSYSTEM.getRobotArea();
    return isPointInRobot(gamePose.getX(), gamePose.getY(), points);
}

private boolean isPointInRobot(double poseX, double posey, double[][] corners) {
    boolean inside = false;
    int previousLength = corners.length - 1;
    int i = 0;
    //here's some fun code. Based on the rotation of the robot it checks if the game piece is inside the robot
    while (i < corners.length) {
        double cornerX = corners[i][0], cornerY = corners[i][1];
        double previousCornerX = corners[previousLength][0], previousCornerY = corners[previousLength][1]; 
        if ((previousCornerY > poseY && cornerY < poseY || previousCornerY < poseY && cornerY > poseY)) {
          double crossingX = previousCornerX + (poseY - previousCornerY) * (cornerX - previousCornerX) / (cornerY - previousCornerY);
          if (poseX < crossingX) {
              inside = !inside;
          }
        }
        previousLength = i;
        i++;
    }
    return inside;
}
    public void runIntoGamePiece(){
      forwardMomentum = DrivetrainDefaultCommand.x2y2return()[0] * -0.1;
      sidewaysMomentum = DrivetrainDefaultCommand.x2y2return()[1] * -0.1;
    }

    public void slowGamePieceMovement(){
      sidewaysMomentum = closerToZero(sidewaysMomentum, forwardMomentum, 0.05);
      forwardMomentum = closerToZero(forwardMomentum, forwardMomentum, 0.05);
      if(sidewaysMomentum > 0 && sidewaysMomentum < 0.05){
        sidewaysMomentum = 0;
      }
      if(forwardMomentum > 0 && forwardMomentum < 0.05){
        forwardMomentum = 0;
      }

    }
    public double closerToZero(double number, double numberToChange, double xCloserToZero){
      if(number == 0){ return number;}
      if(number > 0){
        return numberToChange - xCloserToZero;
      }
      return numberToChange + xCloserToZero;
    }
  
    public void intakeCommand(){
          if(gamePose.getZ() < 0.33){
        poseZ = poseZ + 0.01;
      
      }
      if(Math.abs(poseX - robotX) > 0.01){
        double difference = poseX - robotX;
        poseX = closerToZero(difference, poseX, 0.01);
      
      }
      if(Math.abs(poseY- robotY) > 0.01){
        double difference = poseY - robotY;
        poseY = closerToZero(difference, poseY, 0.01);

      }
      if(Math.abs(poseX) - Math.abs(robotX) < 0.01 && Math.abs(poseY)- Math.abs(robotY) < 0.01 && poseZ- 0.33 < 0.01 && !ConveyerSubsystem.upperSensor){
        inRobot = true;
        ConveyerSubsystem.upperSensor = true;
      }
    }

    public void moveWithRobot(){

      if(Robot.CONVEYER_SUBSYSTEM.getConveyerSpeed() > 0.1 ){
        if(offset < 1){
        offset = offset + 0.01;
        }
        poseZ = conveyerMapZ.get(offset);
        poseY = robotY + conveyerMapY.get(offset);
        poseX = robotX;
        if(offset > 0.99 && poseZ >= 0.71){
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


    public void moveTowardsCenter2(){
      if(isNew){
        //should only be called on the first run of the code
        launchAngle = Math.toRadians(Robot.TURRET_SUBSYSTEM.getPosition() * 36);
        double speed =  2 * Robot.TURRET_SUBSYSTEM.getTurretShootMotorSpeed();
        launchSpeedX = speed * Math.cos(launchAngle);
        launchSpeedY = speed * Math.sin(launchAngle);
        isNew = false;
        upwardsMomentum = speed;
        return;
      }
      poseX = poseX + launchSpeedX;
      poseY = poseY + launchSpeedY;
      launchSpeedX = launchSpeedX * 0.90;
      launchSpeedY = launchSpeedY * 0.90;
      upwardsMomentum = upwardsMomentum -0.05;

      if(launchSpeedX <= 0){
        isNew = true;
        firing = false;
        inTurret = false;
        inRobot = false;
      }
      double shootPoseX =  poseX -8.25; 
      double shootPoseY =  poseY -4.15;
      if(shootPoseY > -0.03 && shootPoseY < 0.03 && shootPoseX > -0.03 && shootPoseX < 0.03){
        firing = false;
        inTurret = false;
        inRobot = false;
        deleteSelf();
      }

    }
    public void moveTowardsCenter(){
      double shootPoseX =  poseX -8.25; 
      double shootPoseY =  poseY -4.15 ;
      if(shootPoseX > 0.03){
        poseX = poseX - 0.05;
      }
      if(shootPoseX < -0.03){
        poseX = poseX + 0.05;
      }
      if(shootPoseY > 0.03){
        poseY = poseY - 0.05;
      }
      if (shootPoseY < -0.03){
        poseY = poseY + 0.05;
      }
      if(shootPoseY > -0.03 && shootPoseY < 0.03 && shootPoseX > -0.03 && shootPoseX < 0.03){
        firing = false;
        inTurret = false;
        inRobot = false;
        deleteSelf();
      }
    }

    public void addThingsToTreeMap(){
      conveyerMapZ.put(0.0,0.33);
      conveyerMapZ.put(0.7, 0.33);
      conveyerMapZ.put(0.8, 0.5);
      conveyerMapZ.put(0.9, 0.7);
      conveyerMapZ.put(1.0, 0.72);

      conveyerMapY.put(0.0, 0.0);
      conveyerMapY.put(0.1, 0.01);
      conveyerMapY.put(0.5, 0.05);
      conveyerMapY.put(0.6, 0.055);
      conveyerMapY.put(0.7, 0.08);
      conveyerMapY.put(0.9, 0.010);
      conveyerMapY.put(1.0,0.115);

    }



    public void deleteSelf(){
      //currently broken, hashing problem while it tries to read/write at the same time
       //Robot.CREATION_CLASS.removeInstance(aprilTagNumber);
       scored = true;
       if(isRed){
       Robot.ARRAY_CLASS.redGamePiecesOnField --;
       }
       else{
        Robot.ARRAY_CLASS.blueGamePiecesOnField--;
       }
    }

    //conditional statements 
    public boolean isInRunningIntakeAndAbleToPickUp(){
      return gamePieceInIntake() && Robot.INTAKE_SUBSYSTEM.getIntakeConveyerSpeed()> 0.1 && !inRobot && isRed == Robot.robotRed && !gamePieceInRobot();
    }
    public boolean ableToFire(){
      return Robot.TURRET_SUBSYSTEM.getTurretShootMotorSpeed() > 0.1 && !firing && inTurret;
    }
    public boolean isNotTouchingRobot(){
      return !gamePieceInRobot()&& !gamePieceInIntake()&& !inRobot;
    }
    public boolean robotOverYEdge(){
      return poseY < Constants.CLOSE_EDGE_Y || poseY > Constants.FAR_EDGE_Y;
    }
    public boolean gamePieceInIntake(){
      double[] numbers = Robot.DRIVE_TRAIN_SIMULATION_SUBSYSTEM.rotatePoint(Robot.DRIVETRAIN_SUBSYSTEM.getPose().getX() + 0.1, Robot.DRIVETRAIN_SUBSYSTEM.getPose().getY()-0.38);
      return Math.abs(gamePose.getX() - numbers[0]) < 0.6 && Math.abs(gamePose.getY() - numbers[1]) < 0.4;
  
    }

    //returns
    public Pose3d returnPose(){
      return gamePose;
    }
    public double[] returnXandY(){
      double[] doubleArray2 = {poseX,poseY,poseZ};
      return doubleArray2;
    }
    public int returnAprilTagId(){
      return aprilTagNumber;
    }

    
}