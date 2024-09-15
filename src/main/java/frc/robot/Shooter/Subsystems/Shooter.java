// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import static frc.robot.Shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX motorUp;
  private TalonFX motorDown;
  private TalonSRX motorFeeding;

  private TalonFXConfiguration config;
  private DutyCycleOut m_request = new DutyCycleOut(0.0);
  private VelocityVoltage velocityVoltage;

  public Shooter() {
    motorFeeding = new TalonSRX(MOTOR_FEEDING_ID);
    motorDown = new TalonFX(MOTOR_DOWN_ID, CANBUS);
    motorUp = new TalonFX(MOTOR_UP_ID, CANBUS);
    config = new TalonFXConfiguration();
    config.Slot0.kP = SHOOTER_KP;
    config.Slot0.kI = SHOOTER_KI;
    config.Slot0.kD = SHOOTER_KD;
    config.Slot0.kS = SHOOTER_KS;
    config.Slot0.kV = SHOOTER_KV;
    config.Voltage.PeakForwardVoltage = 8;
    config.Voltage.PeakReverseVoltage = -8;
    motorUp.getConfigurator().apply(config);
    motorDown.getConfigurator().apply(config);
    velocityVoltage = new VelocityVoltage(0).withSlot(0);
  }

  public void setUpMotorPower(double power){
    motorUp.setControl(m_request.withOutput(power));
  }
  
  public void setDownMotorPower(double power){
    motorDown.setControl(m_request.withOutput(power));
  }

  public void setUpMotorVelocityPid(double desiredRotationsPerSec){
    motorUp.setControl(velocityVoltage.withVelocity(desiredRotationsPerSec));
  }

  public void setDownMotorVelocityPid(double desiredRotationsPerSec){
    motorUp.setControl(velocityVoltage.withVelocity(desiredRotationsPerSec));
  }

  public void setFeedingPower(double power){
    motorFeeding.set(ControlMode.PercentOutput, power);
  }

  public double getDownMotorVelocity(){
    return motorDown.getVelocity().getValue();
  }

  public double getUpMotorVelocity(){
    return motorUp.getVelocity().getValue();
  }

  private enum ShooterState{
    AMP,
    SPEAKER
  }

  public Enum getState(boolean isAmp){
    if(isAmp){
      return ShooterState.AMP;
    }
    return ShooterState.SPEAKER;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
