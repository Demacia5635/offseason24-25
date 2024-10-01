// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class TalonMotor {
  TalonFX talon;
  TalonConfig config;
  TalonFXConfiguration talonConfig;
  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  DutyCycleOut dutyCycle = new DutyCycleOut(0);
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  public TalonMotor(int ID, String CANBUS){
    talon = new TalonFX(ID, CANBUS);
    configMotor();
  }

  private void configMotor(){
    talonConfig = new TalonFXConfiguration();
    talonConfig.CurrentLimits.SupplyCurrentLimit = config.maxCurrent;
    talonConfig.CurrentLimits.SupplyCurrentThreshold = config.maxCurrentTriggerTime;
    talonConfig.CurrentLimits.SupplyTimeThreshold = config.maxCurrentTriggerTime;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.rampUpTime;
    talonConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.rampUpTime;

    talonConfig.MotorOutput.Inverted = config.inverted ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    talonConfig.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    talonConfig.MotorOutput.PeakForwardDutyCycle = config.maxVolt / 12.0;
    talonConfig.MotorOutput.PeakReverseDutyCycle = config.minVolt / 12.0;

    talonConfig.Slot0.kP = config.pid.kp;
    talonConfig.Slot0.kI = config.pid.ki;
    talonConfig.Slot0.kD = config.pid.kd;
    talonConfig.Slot0.kS = config.pid.ks; 
    talonConfig.Slot0.kV = config.pid.kv;
    talonConfig.Slot0.kA = config.pid.ka;
    talonConfig.Slot0.kG = config.pid.kg;
    if(config.pid1 != null) {
      talonConfig.Slot1.kP = config.pid1.kp;
      talonConfig.Slot1.kI = config.pid1.ki;
      talonConfig.Slot1.kD = config.pid1.kd;
      talonConfig.Slot1.kS = config.pid1.ks; 
      talonConfig.Slot1.kV = config.pid1.kv;
      talonConfig.Slot1.kA = config.pid1.ka;
      talonConfig.Slot1.kG = config.pid1.kg;
    }
    if(config.pid2 != null) {
      talonConfig.Slot2.kP = config.pid2.kp;
      talonConfig.Slot2.kI = config.pid2.ki;
      talonConfig.Slot2.kD = config.pid2.kd;
      talonConfig.Slot2.kS = config.pid2.ks; 
      talonConfig.Slot2.kV = config.pid2.kv;
      talonConfig.Slot2.kA = config.pid2.ka;
      talonConfig.Slot2.kG = config.pid2.kg;
    }

    talonConfig.Voltage.PeakForwardVoltage = config.maxVolt;
    talonConfig.Voltage.PeakReverseVoltage = config.minVolt;

    talonConfig.Feedback.SensorToMechanismRatio = config.motorRatio;
    talonConfig.MotionMagic.MotionMagicAcceleration = config.motionMagicAccel;
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = config.motionMagicVelocity;
    talonConfig.MotionMagic.MotionMagicJerk = config.motionMagicJerk;
    talonConfig.MotionMagic.MotionMagicExpo_kA = config.pid.ka;
    talonConfig.MotionMagic.MotionMagicExpo_kV = config.pid.kv;

    velocityVoltage.UpdateFreqHz = 200;
    dutyCycle.UpdateFreqHz = 200;
    motionMagicVoltage.UpdateFreqHz = 200;
    

    talon.getConfigurator().apply(talonConfig);
    talon.getPosition().setUpdateFrequency(200);
    talon.getVelocity().setUpdateFrequency(200);
    talon.getAcceleration().setUpdateFrequency(200);
    talon.getMotorVoltage().setUpdateFrequency(200);
  }

  public void setBrake(boolean isBrake){
    talon.getConfigurator().refresh(talonConfig.MotorOutput);
    talonConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    talon.getConfigurator().apply(talonConfig.MotorOutput);
  }
  
  public void setDuty(double power) {
    talon.setControl(dutyCycle.withOutput(power));
  }

  public void setVelocityRPS(double velocity) {
    talon.setControl(velocityVoltage.withVelocity(velocity));
  }

  public void setMotorPositionRotation(double position){
    talon.setControl(motionMagicVoltage.withPosition(position));  
  }

  public void setMotorPositionRotation2d(Rotation2d position){
    talon.setControl(motionMagicVoltage.withPosition(position.getRotations()));
  }

  public double getPositionRotation() {
    return talon.getPosition().getValue();
  }

  public Rotation2d getPositionRotation2d() {
    return Rotation2d.fromRotations(getPositionRotation());
  }

  public double getVelocityRotationPerSecond() {
    return talon.getVelocity().getValue();
  }
  
  public Rotation2d getVelocityRotation2PerSecond() {
    return Rotation2d.fromRotations(getVelocityRotationPerSecond());
  }
}