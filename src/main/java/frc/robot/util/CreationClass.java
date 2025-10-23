package frc.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

public class CreationClass {
        public GamePiece[] blueCargoArray = {
            new GamePiece(8.25, 8.04,0.13, "one",false, 0),
            new GamePiece(5.50, 6.90,0.13, "two",false,1),
            new GamePiece(4.36, 4.15, 0.13, "three",false,2),
            new GamePiece(11.56, 5.42, 0.13,"four",false,3),
            new GamePiece(4.92, 5.42, 0.13, "five",false,4),
            new GamePiece(9.52, 7.45, 0.13, "six",false,5),
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
    public int numberBlue = 0;
    public int numberRed = 100;
    public int redGamePiecesOnField;
    public int blueGamePiecesOnField;

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
        if(blueGamePiecesOnField < 3 && DriverStation.alliance.get() == Alliance.blue){
            createNewGamePiece(false);
        }
        if(redGamePiecesOnField < 3 && DriverStation.alliance.get() == Alliance.red){
            createNewGamePiece(true);
        }
    }}

    public void onStartup(){
        List<Pose3d> arrayListBlue = new ArrayList<>();
        List<Pose3d> arrayListRed = new ArrayList<>();
        for(int i = 0; i < blueCargoArray.length ; i++){
            arrayListBlue.add(new Pose3d());
            numberBlue++;
            gamePiecesOnField++;
       }
       for(int i = 0; i < redCargoArray.length ; i++){
        arrayListRed.add(new Pose3d());
        numberRed++;
        gamePiecesOnField++;
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

    public void createNewGamePiece(boolean isRed){
        if(!isRed){
        List<GamePiece> arrayListBlue2 = Array.asList(blueCargoArray);
        arrayListBlue2.add(new GamePiece(1,1,"new",false,numberBlue));
        numberBlue++;
        blueCargoArray = arrayListBlue2.toArray();
        List<Pose3d> arrayListBlue1 = Array.asList(cargoArrayBlue);
        arrayListBlue1.add(new Pose3d());
        cargoArrayBlue = arrayListBlue1.toArray();
        Robot.VISION_SUBSYSTEM.addBlueGamePiece();
        }
        if(isRed){
        List<GamePiece> arrayListRed2 = Array.asList(redCargoArray);
        arrayListRed2.add(new GamePiece(1,1,"new",true,numberRed));
        numberRed++;
        blueCargoArray = arrayListRed2.toArray();
        List<Pose3d> arrayListRed1 = Array.asList(cargoArrayRed);
        arrayListRed1.add(new Pose3d());
        cargoArrayRed = arrayListRed1.toArray();
        Robot.VISION_SUBSYSTEM.addRedGamePiece();
        }
        gamePiecesOnField++;
    }
    
}
