// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Constants;

public class ConveyerSubsystem extends SubsystemBase {
  /** Creates a new ConveyerSubsystem. */

  // declare CANs
  private final CANrange _lowerSensor = new CANrange(Constants.LOWER_SENSOR_CAN_ID);
  private final CANrange _upperSensor = new CANrange(Constants.UPPER_SENSOR_CAN_ID);
  private final CANrange _turretSensor = new CANrange(Constants.TURRET_SENSOR_CAN_ID);

  private final LoggedDashboardChooser<Boolean> _lowerSensorChooser = new LoggedDashboardChooser<>("Lower Sensor");
  private final LoggedDashboardChooser<Boolean> _upperSensorChooser = new LoggedDashboardChooser<>("Upper Sensor");
  public final LoggedDashboardChooser<Boolean> _turretSensorChooser = new LoggedDashboardChooser<>("Turret Sensor");

  // declare motors
  public final TalonFX m_conveyerMotorM1 = new TalonFX(Constants.CONVEYER_MOTOR_1_DEVICE_ID);
  public final TalonFX m_conveyerMotorM2 = new TalonFX(Constants.CONVEYER_MOTOR_2_DEVICE_ID);
  private DCMotorSim m_conveyerMotorM1Sim;
  private DCMotorSim m_conveyerMotorM2Sim;

  // have some constants
  private double JKgMetersSquared = 10;
  private double kGearRatio = 0.001;


  public ConveyerSubsystem() {

    if (Robot.isSimulation()) {
      m_conveyerMotorM1Sim = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), JKgMetersSquared, kGearRatio),
          DCMotor.getKrakenX60(1));

      m_conveyerMotorM2Sim = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), JKgMetersSquared, kGearRatio),
          DCMotor.getKrakenX60(1));

      _lowerSensorChooser.addOption("No Ball", false);
      _lowerSensorChooser.addDefaultOption("Has Ball", true);
      _turretSensorChooser.addDefaultOption("No Ball", false);
      _turretSensorChooser.addOption("Has Ball", true);
      _upperSensorChooser.addDefaultOption("No Ball", false);
      _upperSensorChooser.addOption("Has Ball", true);

    }
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

  }

  @Override
  public void periodic() {
    if(Robot.isSimulation()){
      updateSimulation(m_conveyerMotorM1, m_conveyerMotorM1Sim);
      updateSimulation(m_conveyerMotorM2, m_conveyerMotorM2Sim);
    }
    
    //System.out.println(getConveyerSpeed());
  }

  public void moveConveyer(){
    m_conveyerMotorM1.set(0.5);
    m_conveyerMotorM1.set(0.5);
  }

  public void stopConveyer(){
    m_conveyerMotorM1.set(0);
    m_conveyerMotorM2.set(0);
  }
  public void moveConveyerMotorBackwards(){
    m_conveyerMotorM1.set(-0.3);
    m_conveyerMotorM2.set(-0.3);
  }

  public double getConveyerSpeed(){
    return m_conveyerMotorM1.get();
  }

  public boolean ballInturret(){
    if(_turretSensor.getDistance().getValueAsDouble() < 0.5 || _turretSensorChooser.get()){
      return true;
    }
    return false;
  }
  public boolean ballInLowerSensor(){
    if (_lowerSensor.getDistance().getValueAsDouble() > 0.5 || _lowerSensorChooser.get()){
      return true;
    }
    return false;
  }

  public boolean ballinUpperSensor(){
    if (_upperSensor.getDistance().getValueAsDouble() > 0.5 || _upperSensorChooser.get()){
      return true;
    }
    return false;
  }

  public boolean ballRightColor(Alliance ballcolor){
    if(ballcolor == DriverStation.getAlliance().get()){
      return true;
    }
    return false;
  }
  


}
