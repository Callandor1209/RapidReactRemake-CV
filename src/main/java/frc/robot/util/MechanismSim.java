

    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util; 


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.ser.std.ToEmptyObjectSerializer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;


/** Add your docs here. */
public class MechanismSim {

    // variables
    private static Pose3d intakePose = new Pose3d(0.02, -0.248, 0.27, new Rotation3d());
    private static double intakeRotationRand = 0;

    private static Pose3d turretPose = new Pose3d(0, 0.115, 0.72, new Rotation3d());
    //center pointon the field
    //private static Pose3d turretPose = new Pose3d(8.25,4.15,0, new Rotation3d());
    private static double turretRotationRand = 0;

    //Turret methods
    public static void updateTurretRotationChange(double turretRotationChangeRad) {
        turretRotationRand += turretRotationChangeRad;
        updateTurretRotationTotal(turretRotationRand);
    }

    public static void updateTurretRotationTotal(double turretRotationTotalRad) {
        turretRotationRand = turretRotationTotalRad;
        turretPose = new Pose3d(turretPose.getX(), turretPose.getY(), turretPose.getZ(), new Rotation3d(0, 0, turretRotationRand ));
        Logger.recordOutput("MechSim/Turret", turretPose);
    }


    //Intake methods
    public static void updateIntakeRotationChange(double intakeRotationChangeRad) {
        intakeRotationRand += intakeRotationChangeRad;
        updateIntakeRotationTotal(intakeRotationRand);
    }

    public static void updateIntakeRotationTotal(double intakeRotationTotalRad) {
        intakeRotationRand = intakeRotationTotalRad;
        intakePose = new Pose3d(intakePose.getX(), intakePose.getY(), intakePose.getZ(), new Rotation3d( intakeRotationRand, 0, 0));
        Logger.recordOutput("MechSim/Intake", intakePose);
    }

}
