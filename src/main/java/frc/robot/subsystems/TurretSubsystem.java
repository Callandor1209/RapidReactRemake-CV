// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.RotateTurretTowardsCenter;
import frc.robot.util.MechanismSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class TurretSubsystem extends SubsystemBase {

    private final Slot0Configs Slot0Configs = TurretConstants.getSlot0Configs();
  private final TalonFXConfiguration _talonFXConfiguration = this.createTalonFXConfigurationObject();
  public final TalonFX turretMotor = new TalonFX(Constants.TURRET_MOTOR_DEVICE_ID);
  public final TalonFX turretShootMotor = new TalonFX(Constants.TURRET_SHOOT_MOTOR_DEVICE_ID);
  private DCMotorSim m_turretMotorM1; 
  private double kGearRatio = 10;
  private double JKgMetersSquared = .001; // need to find what this actually is
  private boolean defaultCommand = false;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
        if (Robot.isSimulation()) {
      m_turretMotorM1 = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60(1), JKgMetersSquared, kGearRatio),
      DCMotor.getKrakenX60(1));
    }
    turretMotor.getConfigurator().apply(_talonFXConfiguration);

  }

    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\s);
        if (Robot.isSimulation()) {
     updateSimulation(turretMotor, m_turretMotorM1);
    }
  }


  public void goToTargetPosition( double position) {
    turretMotor.setControl(new MotionMagicVoltage(position));
  }

  private TalonFXConfiguration createTalonFXConfigurationObject() {
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration = TurretConstants.MOTION_MAGIC_ACCELERATION;
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    talonFXConfiguration.Slot0 = Slot0Configs;
    return talonFXConfiguration;
  }
  
  public void updateSimulation(TalonFX motor, DCMotorSim simMotor) {
    var talonFXSim2 = motor.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim2.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage2 = talonFXSim2.getMotorVoltage();
    Logger.recordOutput("TurretSubsystem/Motor Volts", motorVoltage2);

    simMotor.setInputVoltage(motorVoltage2);
    simMotor.update(0.020); // Assume 20 ms loop time

    // Get position and velocity
    double mechanismPosition = simMotor.getAngularPositionRotations();
    double mechanismVelocity = simMotor.getAngularVelocityRPM();

    double rotorPosition = mechanismPosition * kGearRatio;
    double rotorVelocity = mechanismVelocity * kGearRatio / 60.0; // Turn into rotations per second

    talonFXSim2.setRawRotorPosition(rotorPosition);
    talonFXSim2.setRotorVelocity(rotorVelocity);

    double mechanismPositionRadians = simMotor.getAngularPositionRotations() * 2 * Math.PI;
    MechanismSim.updateTurretRotationTotal(mechanismPositionRadians);
  }

  public void spinTurretClockwise() {

    turretMotor.set(-0.15);
  }

  public void spinTurretCounterClockwise() {
   turretMotor.set(0.15);
  }

  public void stopTurretSpin() {
    turretMotor.set(0);
  }

  public void shootTurret(){
    turretShootMotor.set(0.3);
  }
  public void stopTurretShoot(){
    turretShootMotor.set(0);
  }

  public double getTurretShootMotorSpeed(){
    return turretShootMotor.get();
  }
  public void setTurretDefault(){
    if(!defaultCommand){
    setDefaultCommand(new RotateTurretTowardsCenter());
    defaultCommand = true;
    return;
    }
    removeDefaultCommand();
    defaultCommand = false;
  }

  public double findCenterAngle(){
    double centerX = 8.25;
    double centerY = 4.15;
     Pose2d pose2d = Robot.DRIVETRAIN_SUBSYSTEM.getState().Pose;
    double turretX = centerX -  pose2d.getX() ;
    double turretY = centerY - pose2d.getY() + 0.115 ;
    double oppositeLegLength = turretY;
    double adjacentLegLength = turretX;
    double targetAngle = Math.atan2(oppositeLegLength, adjacentLegLength);
    targetAngle = targetAngle * 180/Math.PI ;
    targetAngle = targetAngle / 36;
    double robotRotations = pose2d.getRotation().getRotations();
    targetAngle = targetAngle - (robotRotations-0.25) * 10;
    double currentPosition = getPosition();
    double currentAngle = currentPosition/10;
    currentAngle = Math.floor(currentAngle);
    currentAngle = currentAngle * 10;
    currentPosition = currentPosition % 10;
    if (-2.5 < currentPosition && currentPosition < 0  ) {
      if (currentAngle > 0) {
        targetAngle = targetAngle + currentAngle - 10;
        return targetAngle;
      }
      targetAngle = targetAngle + currentAngle + 10;
      return targetAngle;
    }
    targetAngle = targetAngle + currentAngle;
    return targetAngle; 
    
  }
  
  public double getPosition() {
    return turretMotor.getPosition().getValueAsDouble();
}
    @AutoLogOutput
  public double getSpeedM1() {
    return turretMotor.get();
  }

}
