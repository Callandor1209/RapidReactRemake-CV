// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.INTAKE_POSITIONS;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeGoToPositionCommand extends Command {
  private INTAKE_POSITIONS _targetPosition;
  private double targetPosition;



  /** Creates a new ArmGoToAngleCommand. */
  public IntakeGoToPositionCommand(INTAKE_POSITIONS position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.INTAKE_SUBSYSTEM);
    _targetPosition = position;

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     targetPosition = _targetPosition.getPosition();
     System.out.println(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.INTAKE_SUBSYSTEM.goToTargetPosition(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.INTAKE_SUBSYSTEM.goToTargetPosition(targetPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.INTAKE_SUBSYSTEM.atTargetPosition(targetPosition);
  }
}
