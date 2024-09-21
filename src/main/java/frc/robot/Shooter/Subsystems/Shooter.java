// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Shooter.ShooterConstants.MOTOR_IDS;
import frc.robot.Shooter.ShooterConstants.SHOOTER_VAR;
import frc.robot.Shooter.ShooterConstants.STATE;

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
  private DutyCycleOut m_request;
  private VelocityVoltage velocityVoltage;
  public STATE shooterState;
  
  
/*setting all the configs */
  public Shooter() {
    shooterState = shooterState.IDLE;

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

  public void setMotorPower(double power){
    motorUp.setControl(m_request.withOutput(power));
    motorDown.setControl(m_request.withOutput(power));
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




  public void pidMotorVelocity(double vel){
    motorUp.setControl(velocityVoltage.withVelocity(vel));
    motorDown.setControl(velocityVoltage.withVelocity(vel));
}



}
