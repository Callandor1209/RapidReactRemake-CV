// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ConveyerSubsystem;

/*
    Conveyance index logic overview
    Conveyance stage one:
    - If feeder has a ball:
      - If conveyance staging beam break sees a ball
        - If staging beam break ball is proper color
          - If it is, and conveyance entry does NOT see a ball - done
          - If it is, but conveyance entry DOES see a ball - eject the ball until conveyance entry doesn't see anything, and then run the conveyance forward again until the staging beam break sees the ball again
        - If staging beam break ball is NOT proper color
          - Eject until the entry beam break shows true (ball is passing through) and then false (ball has finished passing through) and then done
      - If conveyance staging does NOT see a ball
        - If conveyance entry sees a ball:
          - Move the ball up to the staging bream break (similar to how the feeder waits for a ball from the conveyance - the conveyance staging bream break can "wait" for a ball from the conveyance entry beam break)
          - Once the staging beam break sees the ball, it will be covered by the conditions above that check the color of the ball and eject if necessary
        - If conveyance entry does NOT see a ball (and did not recently see a ball such that staging is "waiting"): there are no balls in the stage 1 conveyance, so nothing needs to be done
    - If the feeder does NOT have a ball:
      - Run the intake and conveyance like normal; the first ball collected should go right to the feeder regardless of its color.
      - The one change to make here is if the ENTRY beam break sees a ball, it needs to index it to the staging beam break (which will then index it to the feeder.)
    */

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ConveyerIndexCommand extends Command {
  ConveyerSubsystem _conveyerSubsystem = Robot.CONVEYER_SUBSYSTEM;
  /** Creates a new ConveyerIndex. */
  public boolean stagingTrue;
  Timer timer;

  public ConveyerIndexCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.CONVEYER_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _conveyerSubsystem.stopConveyer();
    if (_conveyerSubsystem.ballInturret()) {
      if (_conveyerSubsystem.ballinUpperSensor()) {
        // Change to actually use the sensors to figure out what color it is
        if (_conveyerSubsystem.ballRightColor(Alliance.Red)) {

          if (!_conveyerSubsystem.ballInLowerSensor()) {
            return;
          }
          if(_conveyerSubsystem.ballInturret()){
            return;
          }

          if (_conveyerSubsystem.ballInLowerSensor()) {
             _conveyerSubsystem.moveConveyerMotorBackwards();
          }
        } 
        
        else {
          
          timer.start();
          while (!_conveyerSubsystem.ballinUpperSensor() && timer.get() < 5) {
            _conveyerSubsystem.moveConveyerMotorBackwards();
          }
          timer.stop();
          timer.reset();
          return;

          /* feel like this is not needed but I wrote it as it follows what the instructions above said.  
          if (_conveyerSubsystem.ballinUpperSensor()) {
            stagingTrue = true;
            return;
          }
          if (stagingTrue) {
            stagingTrue = false;
            _conveyerSubsystem.stopConveyer();
        }
            */
          }

        }
        else{
          if (!_conveyerSubsystem.ballInLowerSensor()) {
            return;
          }
          if (_conveyerSubsystem.ballInLowerSensor()) {

            timer.start();
            while (!_conveyerSubsystem.ballinUpperSensor() && timer.get() < 5) {
              _conveyerSubsystem.moveConveyer();
            }
            timer.stop();
            timer.reset();
            return;
          }
        }
      }

    }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
