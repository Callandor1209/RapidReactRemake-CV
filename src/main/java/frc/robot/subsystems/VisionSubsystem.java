// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSubsystem extends SubsystemBase {
  VisionSystemSim visionSim;
  SimCameraProperties cameraProperties = new SimCameraProperties();
  PhotonCamera frontCamera;
  PhotonCameraSim cameraSim;
  int i;
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    if(Robot.isSimulation()){
     visionSim = new VisionSystemSim("CameraSimSystem");
     frontCamera = new PhotonCamera("Camera");
     cameraSim = new PhotonCameraSim(frontCamera, cameraProperties);
     visionSim.addCamera(cameraSim, new Transform3d());


    }
  }
  public void simUpdate(){
    visionSim.update(Robot.DRIVETRAIN_SUBSYSTEM.getPose());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    i = RobotContainer.CREATION_CLASS.creationArray.length;
    while(i > 0){
    RobotContainer.CREATION_CLASS.creationArray[i].returnXandY();
    i--;
    }
  }
}
