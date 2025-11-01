package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;

public class Wall{
    double slopeX;
    double slopeY;
    double robotX;
    double robotY;
    double b;
    double b2;
    double slopeX2;
    double slopeY2;
    double distance;
    double minimumDistance = 0.5;
    double startPointX;
    double endPointX;
    double startPointY;
    double endPointY;

    //ok so basically do the maths where you find the point of intersection by doing the perpendicular stuffs and then do the distance formula 
    public Wall(Pose2d startPoint, Pose2d  endPoint){
        startPointX = startPoint.getX();
        startPointY = startPoint.getY();
        endPointX = endPoint.getX();
        endPointY = endPoint.getY();
        slopeX = startPoint.getX() - endPoint.getX();
        slopeY = startPoint.getY() - endPoint.getY();
        b = getB(slopeX,slopeY,startPoint.getX(),startPoint.getY());
        System.out.println("Startup Succesfull");
    }

    public double getB(double x, double y, double pointX, double pointY ){
        return (pointY - ((y/x) * pointX));
    }
    public void periodic(){
        
        robotX = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getX();
        robotY = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getY();
        if(((robotX > startPointX && robotX > endPointX) && (robotY > endPointY && robotY > endPointY))||((robotX < startPointX && robotX < endPointX) && (robotY > endPointY && robotY > endPointY)) || ((robotX < startPointX && robotX < endPointX) && (robotY < endPointY && robotY < endPointY))||((robotX > startPointX && robotX > endPointX) && (robotY < endPointY && robotY < endPointY))){
            return;
        }
        distance = findDistance();

        if(distance < minimumDistance ){
//invert speeds
            ChassisSpeeds currentSpeeds = Robot.DRIVETRAIN_SUBSYSTEM.getState().Speeds;
            double newX = -currentSpeeds.vxMetersPerSecond * 0.5;
            double newY = -currentSpeeds.vyMetersPerSecond *0.5;
            ChassisSpeeds newSpeeds = new ChassisSpeeds(newX,newY,0);
            Robot.DRIVETRAIN_SUBSYSTEM.driveInDirection(newSpeeds);
        }
    }

    public double findDistance(){
        double oppositeX = -slopeY;
        double oppositeY = slopeX;
        b2 = getB(oppositeX,oppositeY, robotX, robotY);
        double b3 = b2-b;
        double coeffeciant = slopeY/slopeX - oppositeY/oppositeX;
        double answerX = b3 / coeffeciant;
        double answerY = answerX * (slopeY/slopeX) + b;
        double actualDistance = Math.sqrt((robotX - answerX)*(robotX - answerX)+(robotY-answerY)*(robotY-answerY));
        return actualDistance;

    }
}