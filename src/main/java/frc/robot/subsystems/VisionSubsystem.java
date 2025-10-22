// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  public VisionSystemSim visionSim;
  SimCameraProperties cameraProperties = new SimCameraProperties();
  PhotonCamera frontCamera;
  PhotonCameraSim cameraSim;
  double targetYaw;
  List<VisionTargetSim> targetSimlistRed = new ArrayList<>();
  List<VisionTargetSim> targetSimlistBlue = new ArrayList<>();
  VisionTargetSim[] visionTargetArrayBlue;
  VisionTargetSim[] visionTargetArrayRed;

  int i;
  int x = 0;

  double yaw = 0;
  double area;
  int tagId;

  TargetModel targetModel = new TargetModel(0.5, 0.25);
        Pose3d targetPose = new Pose3d(0,0,0, new Rotation3d());

  public VisionSubsystem() {
    if(Robot.isSimulation()){
     visionSim = new VisionSystemSim("CameraSimSystem");
     frontCamera = new PhotonCamera("Camera");
     cameraSim = new PhotonCameraSim(frontCamera, cameraProperties);
  
     visionSim.addCamera(cameraSim, new Transform3d(0,0,0, new Rotation3d(0,0,Math.toRadians(270))));
cameraSim.enableRawStream(true);
cameraSim.enableProcessedStream(true);

cameraSim.enableDrawWireframe(true);

for(int i = 0; i < Robot.CREATION_CLASS.blueCargoArray.length; i++){
    targetSimlistBlue.add(new VisionTargetSim(targetPose, targetModel,i));
}
visionTargetArrayBlue = targetSimlistBlue.toArray(new VisionTargetSim[0]);
for(int i = 0; i < Robot.CREATION_CLASS.redCargoArray.length; i++){
  targetSimlistRed.add(new VisionTargetSim(targetPose, targetModel,i + 100));
}
visionTargetArrayRed = targetSimlistRed.toArray(new VisionTargetSim[0]);
visionTargetArrayBlue = targetSimlistBlue.toArray(new VisionTargetSim[0]);
  }
}

  public void simUpdate() {
    visionSim.update(Robot.DRIVETRAIN_SUBSYSTEM.getPose());

  }

  @Override
  public void periodic() {
    if(!DriverStation.isEnabled()){
      return;
    }
    // This method will be called once per scheduler run
    int i2 = Robot.CREATION_CLASS.blueCargoArray.length -1 ;
    //once, when the robot first activates the vision targets are added to the sim
    while (i2 >= 0 && DriverStation.isEnabled() && x == 0 && Robot.isSimulation()) {
      visionSim.addVisionTargets(visionTargetArrayBlue[i2]);
      i2--;
    }
     i2 = Robot.CREATION_CLASS.redCargoArray.length - 1;
    while (i2 >= 0 && DriverStation.isEnabled() && x == 0 && Robot.isSimulation()) {
      visionSim.addVisionTargets(visionTargetArrayRed[i2]);
      i2--;
    }
      if(x==5){
        updateTargets();
        x= 1;
      }
      x++;
      


  
    List<PhotonPipelineResult>  pipelineResult = frontCamera.getAllUnreadResults();
    if(!pipelineResult.isEmpty()){
    PhotonPipelineResult singularResult = pipelineResult.get(pipelineResult.size() - 1);
    if(singularResult.hasTargets()){
    PhotonTrackedTarget bestTarget = singularResult.getBestTarget();
     targetYaw = bestTarget.getYaw();
      area = bestTarget.getArea();
      tagId = bestTarget.getFiducialId();
    }
    else{
      targetYaw = 0;
    }
  }
  


  }

  public int returnTagId(){
    return tagId;
  }

  public double returnYaw(){
    return targetYaw;
  }

  public void updateTargets(){
    i = Robot.CREATION_CLASS.blueCargoArray.length -1;
   yaw = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getRotation().getRadians() + Math.toRadians(90);
    while (i >= 0 && DriverStation.isEnabled() && x > 0 && Robot.isSimulation()) {
      double[] xyz = Robot.CREATION_CLASS.blueCargoArray[i].returnXandY();

      Pose3d targetPose = new Pose3d(xyz[0], xyz[1], xyz[2], new Rotation3d(0, 0, yaw));
      visionTargetArrayBlue[i].setPose(targetPose);
      i--;
      
    }
    i = Robot.CREATION_CLASS.redCargoArray.length - 1;
    while (i >= 0 && DriverStation.isEnabled() && x > 0 && Robot.isSimulation()) {
      double[] xyz = Robot.CREATION_CLASS.redCargoArray[i].returnXandY();

      Pose3d targetPose = new Pose3d(xyz[0], xyz[1], xyz[2], new Rotation3d(0, 0, yaw));
      visionTargetArrayRed[i].setPose(targetPose);
      i--;
    }
  }

  public double returnArea(){
    return area;
  }
}
