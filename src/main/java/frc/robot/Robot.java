// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DrivetrainDefaultCommand;
import frc.robot.commands.PutBallInTurretCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.MechanismSim;
import frc.robot.util.TunerConstants;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {

  
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
    public static final TurretSubsystem TURRET_SUBSYSTEM = new TurretSubsystem();
    public static final CommandSwerveDrivetrain DRIVETRAIN_SUBSYSTEM = TunerConstants.createDrivetrain();
    public static final IntakeSubsystem INTAKE_SUBSYSTEM = new IntakeSubsystem();
    public static final ConveyerSubsystem CONVEYER_SUBSYSTEM = new ConveyerSubsystem();
    public static final SwerveRequest.ApplyRobotSpeeds PATH_APPLY_ROBOT_SPEEDS = new SwerveRequest.ApplyRobotSpeeds();

      public static final SwerveRequest.FieldCentric SWERVE_REQUEST_DRIVE = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

if (isReal()) {
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
} else {
  Logger.addDataReceiver(new NT4Publisher()); // Publish to NetworkTables for live viewing
}

Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

MechanismSim.updateIntakeRotationTotal(0);
MechanismSim.updateTurretRotationTotal(0);
Pose3d ballPose = new Pose3d(0, 0, 0, new Rotation3d());
Logger.recordOutput("component/pose3d", ballPose);


    m_robotContainer = new RobotContainer();
    CONVEYER_SUBSYSTEM.setDefaultCommand(new PutBallInTurretCommand());
    DRIVETRAIN_SUBSYSTEM.setDefaultCommand(new DrivetrainDefaultCommand());
  
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
        Pose2d pose2d = DRIVETRAIN_SUBSYSTEM.getState().Pose;
    Pose3d pose3d = new Pose3d(
        pose2d.getX(), 
        pose2d.getY(), 
        0.0,
        new Rotation3d(0, 0, pose2d.getRotation().getRadians()));
        Logger.recordOutput("Robot/Pose3d", pose3d);


    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
