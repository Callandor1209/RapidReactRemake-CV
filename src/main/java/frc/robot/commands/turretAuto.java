// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAuto extends Command {
  /** Creates a new shootPieceAuto. */
  public TurretAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.INTAKE_SUBSYSTEM);
  }

  private boolean done1 = false;
  private boolean done2 = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.INTAKE_SUBSYSTEM.startIntakeConveyer();
    if(!Robot.CONVEYER_SUBSYSTEM.ballInturret()){
      return;
    }
    if(Robot.TURRET_SUBSYSTEM.getSpeedM1() == 0){
      Robot.TURRET_SUBSYSTEM.shootTurret();
      done1 = true;
    }
    Robot.CONVEYER_SUBSYSTEM.stopConveyer();

    if(done1 && !Robot.CONVEYER_SUBSYSTEM.ballInturret()){

        done2 = true;


    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.TURRET_SUBSYSTEM.stopTurretShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done2;
  }
}
