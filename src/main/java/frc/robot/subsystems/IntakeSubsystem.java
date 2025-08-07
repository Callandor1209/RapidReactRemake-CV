// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.MechanismSim;

public class IntakeSubsystem extends SubsystemBase {

  private final Slot0Configs Slot0Configs = IntakeConstants.getSlot0Configs();
  private final TalonFXConfiguration _talonFXConfiguration = this.createTalonFXConfigurationObject();
  public final TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_DEVICE_ID);
  public final TalonFX intakeConveyerMotor = new TalonFX(Constants.INTAKE_MOTOR_CONVEYER_DEVICE_ID);
  private DCMotorSim m_intakeMotorM1; 
  private double kGearRatio = 10;
  private double JKgMetersSquared = .001; // need to find what this actually is

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
     if (Robot.isSimulation()) {
      m_intakeMotorM1 = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60(1), JKgMetersSquared, kGearRatio),
      DCMotor.getKrakenX60(1));
    }
    intakeMotor.getConfigurator().apply(_talonFXConfiguration);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      if (Robot.isSimulation()) {
     updateSimulation(intakeMotor, m_intakeMotorM1);
    }
  }
  public void updateSimulation(TalonFX motor, DCMotorSim simMotor) {
    var talonFXSim2 = motor.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim2.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage2 = talonFXSim2.getMotorVoltage();
    Logger.recordOutput("IntakeSubsystem/Motor Volts", motorVoltage2);

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
    MechanismSim.updateIntakeRotationTotal(mechanismPositionRadians);
  }



   private TalonFXConfiguration createTalonFXConfigurationObject() {
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration = IntakeConstants.MOTION_MAGIC_ACCELERATION;
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    talonFXConfiguration.Slot0 = Slot0Configs;
    return talonFXConfiguration;
  }


   public void goToTargetPosition( double position) {
 
    intakeMotor.setControl(new MotionMagicVoltage(position));
  }

  public double getMotorSpeed(){
    return intakeMotor.get();
  }
  public double getIntakeConveyerSpeed(){
    return intakeConveyerMotor.get();
  }
  public void moveIntakeConveyerMotorBackwards(){
    intakeConveyerMotor.set(-0.3);
  }



  public void stopIntakeRotation(){
    intakeMotor.set(0);
  }
  public double getPosition() {
    return intakeMotor.getPosition().getValueAsDouble();
}
public boolean atTargetPosition(double targetPosition) {
  return Math.abs(getPosition() - targetPosition) < 0.1;
}
public void startIntakeConveyer(){
  intakeConveyerMotor.set(0.3);
}
public void stopIntakeConveyermoter(){
intakeConveyerMotor.set(0.0);
}
}
