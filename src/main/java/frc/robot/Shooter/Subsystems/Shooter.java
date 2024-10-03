// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.ShooterConstants.MOTOR_IDS;
import frc.robot.Shooter.ShooterConstants.SHOOTER_VAR;
import frc.robot.Shooter.ShooterConstants.STATE;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {
  
  private TalonFX motorUp;
  private TalonFX motorDown;
  private TalonSRX motorFeeding;

  private TalonFXConfiguration config;
  private DutyCycleOut m_request;
  private VelocityVoltage velocityVoltage;
  public STATE shooterState;
  
  /** Creates a new Shooter. */
  public Shooter() {
    shooterState = STATE.SPEAKER;

    motorFeeding = new TalonSRX(MOTOR_IDS.MOTOR_FEEDING_ID);
    motorDown = new TalonFX(MOTOR_IDS.MOTOR_DOWN_ID, MOTOR_IDS.CANBUS);
    motorUp = new TalonFX(MOTOR_IDS.MOTOR_UP_ID, MOTOR_IDS.CANBUS);

    m_request = new DutyCycleOut(0.0);
    velocityVoltage = new VelocityVoltage(0).withSlot(0);
    config = new TalonFXConfiguration();
    

    config.Slot0.kP = SHOOTER_VAR.KP;
    config.Slot0.kI = SHOOTER_VAR.KI;
    config.Slot0.kD = SHOOTER_VAR.KD;
    config.Slot0.kS = SHOOTER_VAR.KS;
    config.Slot0.kV = SHOOTER_VAR.KV;

    

    motorUp.getConfigurator().apply(config);
    motorDown.getConfigurator().apply(config);

    
  }

  public void setMotorPower(double upPower, double downPower){
    motorUp.setControl(m_request.withOutput(upPower));
    motorDown.setControl(m_request.withOutput(downPower));
  }
  


  public void setFeedingPower(double power){
    motorFeeding.set(ControlMode.PercentOutput, power);
  }

  public double getDownMotorVel(){
    return motorDown.getVelocity().getValue();
  }

  public double getUpMotorVel(){
    return motorUp.getVelocity().getValue();
  }

  public void pidMotorVelocity(double upVel, double downVel){
    motorUp.setControl(velocityVoltage.withVelocity(upVel));
    motorDown.setControl(velocityVoltage.withVelocity(downVel));
  }


  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("upMotorVel", getUpMotorVel());
    SmartDashboard.putNumber("downMotorVel", getDownMotorVel());
  }
}
