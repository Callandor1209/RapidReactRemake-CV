package frc.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

public class CreationClass {
        public GamePiece[] blueCargoArray = {
            new GamePiece(8.25, 8.04, "one",false, 0),
            new GamePiece(5.50, 6.90, "two",false,1),
            new GamePiece(4.36, 4.15, "three",false,2),
            new GamePiece(11.56, 5.42, "four",false,3),
            new GamePiece(4.92, 5.42, "five",false,4),
            new GamePiece(9.52, 7.45, "six",false,5),
        };
        public GamePiece[] redCargoArray = {
            new GamePiece(8.25, 0.26,"oneR",true,100),
            new GamePiece(11.00, 1.40, "twoR",true,101),
            new GamePiece(12.14, 4.15,"threeR",true,102),
            new GamePiece(11.00, 6.905,"fourR",true,103),
            new GamePiece(5.50, 1.40,"fiveR",true,104),
            new GamePiece(4.94, 2.88,"sixR",true,105)};
        
    public Pose3d[] cargoArrayRed;
    public Pose3d[] cargoArrayBlue;

    public void logGamePieces(){{
        if(!DriverStation.isEnabled()){
            return;
        }
        for(int i = 0; i < blueCargoArray.length ; i++){
             cargoArrayBlue[i] = blueCargoArray[i].returnPose();
        }
        for(int i = 0; i < redCargoArray.length; i++){
            cargoArrayRed[i] = redCargoArray[i].returnPose();
        }
        Logger.recordOutput("Cargo Simulation/Cargo Array Blue", cargoArrayBlue);
        Logger.recordOutput("Cargo Simulation/Cargo Array Red", cargoArrayRed);
    }}

    public void onStartup(){
        List<Pose3d> arrayListBlue = new ArrayList<>();
        List<Pose3d> arrayListRed = new ArrayList<>();
        for(int i = 0; i < blueCargoArray.length ; i++){
            arrayListBlue.add(new Pose3d());
       }
       for(int i = 0; i < redCargoArray.length ; i++){
        arrayListRed.add(new Pose3d());
       }
       cargoArrayBlue = arrayListBlue.toArray(new Pose3d[0]);
       cargoArrayRed = arrayListRed.toArray(new Pose3d[0]);
    }

    public void removeInstance(int id){
        if(id < 100){
        int i = 0;
        for(int l = 0; id != blueCargoArray[l].returnAprilTagId(); l++ ){
            i = l;
        }
        blueCargoArray[i] = new GamePiece(-100, -100, "scored", false, 1000);;
    }
    else{
        int i = 0; 
        for(int l = 0; id != redCargoArray[l].returnAprilTagId(); l++){
            i=l;
        }
        redCargoArray[i] = new GamePiece(-100, -100, "scored", false, 1000);
    }
    }
}
