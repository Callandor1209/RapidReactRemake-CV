package frc.robot.util;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;

public class ArrayClass {
        public GamePiece[] blueCargoArray = {
            new GamePiece(8.25, 8.04,0.13, "one",false, 0),
            new GamePiece(5.50, 6.90,0.13, "two",false,1),
            new GamePiece(4.36, 4.15, 0.13, "three",false,2),
            new GamePiece(11.56, 5.42, 0.13,"four",false,3),
            new GamePiece(4.92, 5.42, 0.13, "five",false,4),
            new GamePiece(9.52, 7.45, 0.13, "six",false,5),
        };
        public GamePiece[] redCargoArray = {
            new GamePiece(8.25, 0.26, 0.13,"oneR",true,100),
            new GamePiece(11.00, 1.40, 0.13 , "twoR",true,101),
            new GamePiece(12.14, 4.15,0.13,"threeR",true,102),
            new GamePiece(11.00, 6.905, 0.13,"fourR",true,103),
            new GamePiece(5.50, 1.40,0.13,"fiveR",true,104),
            new GamePiece(4.94, 2.88,0.13,"sixR",true,105)};
        
    public Pose3d[] cargoArrayRed;
    public Pose3d[] cargoArrayBlue;
    public int numberBlue = 0;
    public int numberRed = 100;
    public int redGamePiecesOnField;
    public int blueGamePiecesOnField;
    public double x = 0;
    public double y = 0;
    public int reverseValue = 1;

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
        if(blueGamePiecesOnField < 3 && DriverStation.getAlliance().get() == Alliance.Blue){
            createNewGamePiece(false);
        }
        if(redGamePiecesOnField < 3 && DriverStation.getAlliance().get() == Alliance.Red){
            createNewGamePiece(true);
        }
    }}

    public void onStartup(){
        List<Pose3d> arrayListBlue = new ArrayList<>();
        List<Pose3d> arrayListRed = new ArrayList<>();
        for(int i = 0; i < blueCargoArray.length ; i++){
            arrayListBlue.add(new Pose3d());
            numberBlue++;
            blueGamePiecesOnField++;
       }
       for(int i = 0; i < redCargoArray.length ; i++){
        arrayListRed.add(new Pose3d());
        numberRed++;
        redGamePiecesOnField++;
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
        blueCargoArray[i] = new GamePiece(-100, -100, 5, "scored", false, 1000);;
    }
    else{
        int i = 0; 
        for(int l = 0; id != redCargoArray[l].returnAprilTagId(); l++){
            i=l;
        }
        redCargoArray[i] = new GamePiece(-100, -100, 5, "scored", false, 1000);
    }
    }

    public void createNewGamePiece(boolean isRed){
        if(!isRed){
            x = Math.random() * 20;
            y = Math.random() * 10;
            if(y > 8){
                y = 2;
            }
            if(x> 15){
                x = 2;
            }
        List<GamePiece> arrayListBlue2 = new ArrayList<>(Arrays.asList(blueCargoArray));
        arrayListBlue2.add(new GamePiece(x,y,5,"new",false,numberBlue));

        blueCargoArray = arrayListBlue2.toArray(new GamePiece[0]);
        List<Pose3d> arrayListBlue1 = new ArrayList<>(Arrays.asList(cargoArrayBlue));
        arrayListBlue1.add(new Pose3d());
        cargoArrayBlue = arrayListBlue1.toArray(new Pose3d[0]);
        Robot.VISION_SUBSYSTEM.addBlueGamePiece(numberBlue);
        blueGamePiecesOnField++;
        numberBlue++;
        }
        if(isRed){
            x = Math.random() * 10;
            y = Math.random() * 10;
            if(y > 8){
                y = 2;
            }
            if(x> 15){
                x = 2;
            }
        List<GamePiece> arrayListRed2 = new ArrayList<>(Arrays.asList(redCargoArray));
        arrayListRed2.add(new GamePiece(x,y,5,"new",true,numberRed));

        redCargoArray = arrayListRed2.toArray(new GamePiece[0]);
        List<Pose3d> arrayListRed1 = new ArrayList<>(Arrays.asList(cargoArrayRed));
        arrayListRed1.add(new Pose3d());
        cargoArrayRed = arrayListRed1.toArray(new Pose3d[0]);
        Robot.VISION_SUBSYSTEM.addRedGamePiece(numberRed);
        redGamePiecesOnField++;
        numberRed++;
        }

    }
    
}
