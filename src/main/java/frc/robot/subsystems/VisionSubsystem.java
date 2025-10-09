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
import frc.robot.RobotContainer;

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
  double distanceX;
  double distanceY;

  int i;
  int x = 0;

  TargetModel targetModel = new TargetModel(0.5, 0.25);
        Pose3d targetPose = new Pose3d(0,0,0, new Rotation3d());
        VisionTargetSim[] visionTargetArray = {
      new VisionTargetSim(targetPose, targetModel,1),
      new VisionTargetSim(targetPose, targetModel,2),
      new VisionTargetSim(targetPose, targetModel,3),
      new VisionTargetSim(targetPose, targetModel,4), 
      new VisionTargetSim(targetPose, targetModel,5),
      new VisionTargetSim(targetPose, targetModel,6),
      new VisionTargetSim(targetPose, targetModel,7),
      new VisionTargetSim(targetPose, targetModel,8),
      new VisionTargetSim(targetPose, targetModel,9),
      new VisionTargetSim(targetPose, targetModel,10), 
      new VisionTargetSim(targetPose, targetModel,11),
      new VisionTargetSim(targetPose, targetModel,12)
      };

  public VisionSubsystem() {
    if(Robot.isSimulation()){
     visionSim = new VisionSystemSim("CameraSimSystem");
     frontCamera = new PhotonCamera("Camera");
     cameraSim = new PhotonCameraSim(frontCamera, cameraProperties);
  
     visionSim.addCamera(cameraSim, new Transform3d(0,0,0, new Rotation3d(0,0,Math.toRadians(270))));
cameraSim.enableRawStream(true);
cameraSim.enableProcessedStream(true);

cameraSim.enableDrawWireframe(true);

  }
  }

  public void simUpdate() {
    visionSim.update(Robot.DRIVETRAIN_SUBSYSTEM.getPose());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    i = RobotContainer.CREATION_CLASS.creationArray.length - 1;

    while (i >= 0 && DriverStation.isEnabled() && x == 0 && Robot.isSimulation()) {

     

      visionSim.addVisionTargets(visionTargetArray[i]);

      if (i == 0) {
        x++;
      }

      i--;

    }
    switch(x){
      case 1:
        x = 2;
        break;
      case 2:
        x =3;
        break;
      case 3:
        x =4;
        break;
      case 4:
        updateTargets();
        x = 1;
        break;
      default: break;
    }
    List<PhotonPipelineResult>  pipelineResult = frontCamera.getAllUnreadResults();
    if(!pipelineResult.isEmpty()){
    PhotonPipelineResult singularResult = pipelineResult.get(pipelineResult.size() - 1);
    if(singularResult.hasTargets()){
    PhotonTrackedTarget bestTarget = singularResult.getBestTarget();
     distanceX = bestTarget.getBestCameraToTarget().getTranslation().getX();
     distanceY = bestTarget.getBestCameraToTarget().getTranslation().getY();
    System.out.println(distanceX + " x, y " + distanceY );
    }
    else{
      distanceX = 0;
      distanceY = 0;
    }
    }


  }

  public double[] returnXandYTranslation(){
    double[] doubleArray5 = {distanceX, distanceY};
    return doubleArray5;
  }

  public void updateTargets(){
    while (i >= 0 && DriverStation.isEnabled() && x > 0 && Robot.isSimulation()) {
      double[] xyz = RobotContainer.CREATION_CLASS.creationArray[i].returnXandY();
      Pose3d targetPose = new Pose3d(xyz[0], xyz[1], xyz[2], new Rotation3d(0, 0, 0));
      visionTargetArray[i].setPose(targetPose);
      i--;
      
    }
  }
}
