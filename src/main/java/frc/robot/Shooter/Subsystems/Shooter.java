// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.ShooterConstants.MOTOR_IDS;
import frc.robot.Shooter.ShooterConstants.SHOOTER_VAR;
import frc.robot.Shooter.ShooterConstants.STATE;

import static frc.robot.Shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  
  private TalonFX motorUp;
  private TalonFX motorDown;
  private TalonSRX motorFeeding;

  private TalonFXConfiguration configShooting;
  private TalonFXConfiguration configUp;
  private TalonFXConfiguration configDown;

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
    configShooting = new TalonFXConfiguration();
    
    configShooting.Slot0.kP = SHOOTER_VAR.KP;
    configShooting.Slot0.kI = SHOOTER_VAR.KI;
    configShooting.Slot0.kD = SHOOTER_VAR.KD;
    configShooting.Slot0.kS = SHOOTER_VAR.KS;
    configShooting.Slot0.kV = SHOOTER_VAR.KV;
    
    configShooting.MotorOutput.NeutralMode = IS_SHOOTING_MOTORS_BRAKE
    ? NeutralModeValue.Brake
    : NeutralModeValue.Coast;

    configDown = configShooting;
    configUp = configShooting;    

    configUp.MotorOutput.Inverted = IS_UP_MOTOR_INVERT ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    configDown.MotorOutput.Inverted = IS_DOWN_MOTOR_INVERT ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motorUp.getConfigurator().apply(configUp);
    motorDown.getConfigurator().apply(configDown);
    
    motorFeeding.configFactoryDefault();
    motorFeeding.setInverted(IS_FEEDING_MOTOR_INVERT);
    motorFeeding.setNeutralMode(IS_FEEDING_MOTOR_BRAKE ? NeutralMode.Brake : NeutralMode.Coast);
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
