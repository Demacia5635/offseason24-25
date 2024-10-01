// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class TalonMotor {
  TalonFX talon;
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