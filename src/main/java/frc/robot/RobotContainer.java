// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Constants.OperatorConstants;
import frc.robot.Constants.IntakeConstants.INTAKE_POSITIONS;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTwoInchesInDirection;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.commands.IntakeGoToPositionCommand;
import frc.robot.commands.MoveIntakeConveyerMotorCommand;
import frc.robot.commands.MoveObjectBackwardsInConveyer;
import frc.robot.commands.SpinTurretClockwise;
import frc.robot.commands.SpinTurretCounterClockwise;
import frc.robot.commands.SpinTurretShootMotorCommand;
import frc.robot.commands.RotateTurretTowardsCenter;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.MechanismSim;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.util.DriveFeedforwards;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...





  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

    public final static CommandXboxController M_XBOX_CONTROLLER = new CommandXboxController(OperatorConstants.kDriverControllerPort );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
/* 
    //Turret triggers
    new Trigger(() -> m_driverController.getL2Axis() > 0.1).whileTrue(new SpinTurretCounterClockwise());
    new Trigger(() -> m_driverController.getR2Axis() > 0.1).whileTrue(new SpinTurretClockwise());
    //Intake triggers
    m_driverController.triangle().onTrue(new IntakeGoToPositionCommand(INTAKE_POSITIONS.INTAKE_POSITION_UP));
    m_driverController.cross().onTrue(new IntakeGoToPositionCommand(INTAKE_POSITIONS.INTAKE_POSITION_DOWN));
    //new Trigger(() -> Math.abs(m_driverController.getLeftY()) > 0.1 ||Math.abs(m_driverController.getLeftX()) > 0.1 ||  Math.abs(m_driverController.getRightX()) > 0.1  || Math.abs(m_driverController.getRightY()) >0.1).whileTrue(new DrivetrainDefaultCommand());
    new Trigger(() -> m_driverController.getR2Axis() > 0.9 && m_driverController.getL2Axis() > 0.99).onTrue(new Command(){
      public void initialize(){
        if (Robot.isSimulation()) {
          Robot.TURRET_SUBSYSTEM.setTurretDefault();
        }
      }
      public boolean isFinished(){
        return true;
      }
    });
    m_driverController.R1().whileTrue(new MoveIntakeConveyerMotorCommand());
    m_driverController.square().whileTrue(new SpinTurretShootMotorCommand());
    m_driverController.circle().whileTrue(new MoveObjectBackwardsInConveyer());
    */


 
    //xbox controller triggers
    new Trigger(() -> M_XBOX_CONTROLLER.getLeftTriggerAxis() > 0.1).whileTrue(new SpinTurretCounterClockwise());
    new Trigger(() -> M_XBOX_CONTROLLER.getRightTriggerAxis() > 0.1).whileTrue(new SpinTurretClockwise());
    M_XBOX_CONTROLLER.y().onTrue(new IntakeGoToPositionCommand(INTAKE_POSITIONS.INTAKE_POSITION_UP));
    M_XBOX_CONTROLLER.a().onTrue(new IntakeGoToPositionCommand(INTAKE_POSITIONS.INTAKE_POSITION_DOWN));
    new Trigger(() -> M_XBOX_CONTROLLER.getRightTriggerAxis() > 0.9 && M_XBOX_CONTROLLER.getLeftTriggerAxis() > 0.99).onTrue(new Command(){
      public void initialize(){
        if (Robot.isSimulation()) {
          Robot.TURRET_SUBSYSTEM.setTurretDefault();
        }
      }
      public boolean isFinished(){
        return true;
      }
    });
    M_XBOX_CONTROLLER.rightBumper().whileTrue(new MoveIntakeConveyerMotorCommand());
    M_XBOX_CONTROLLER.x().whileTrue(new SpinTurretShootMotorCommand());
    M_XBOX_CONTROLLER.b().whileTrue(new MoveObjectBackwardsInConveyer());
    M_XBOX_CONTROLLER.povUp().onTrue(new DriveTwoInchesInDirection('F'));
    M_XBOX_CONTROLLER.povDown().onTrue(new DriveTwoInchesInDirection('B'));
    M_XBOX_CONTROLLER.povLeft().onTrue(new DriveTwoInchesInDirection('L'));
    M_XBOX_CONTROLLER.povRight().onTrue(new DriveTwoInchesInDirection('R'));

    
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
  //}
}
