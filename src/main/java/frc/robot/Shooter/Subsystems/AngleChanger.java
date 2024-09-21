// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_CONFIGS;
import frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_VAR;
import frc.robot.Shooter.ShooterConstants.MOTOR_IDS;
import frc.robot.Shooter.ShooterConstants.STATE;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import static frc.robot.Shooter.ShooterConstants.*;

public class AngleChanger extends SubsystemBase {
  
  private TalonFX angleChangingMotor;

  private TalonFXConfiguration config;
  private DutyCycleOut m_request;
  private VelocityVoltage velocityVoltage;
  private MotionMagicVoltage motionMagicVoltage;
  public STATE angleState;


  /** Creates a new AngleChanging. */
  public AngleChanger() {

    angleChangingMotor = new TalonFX(MOTOR_IDS.ANGLE_CHANGING_ID, MOTOR_IDS.CANBUS);
    config = new TalonFXConfiguration();
    angleState = angleState.IDLE;

    config.Slot0.kP = ANGLE_CHANGING_CONFIGS.KP;
    config.Slot0.kI = ANGLE_CHANGING_CONFIGS.KI;
    config.Slot0.kD = ANGLE_CHANGING_CONFIGS.KD;
    config.Slot0.kS = ANGLE_CHANGING_CONFIGS.KS;
    config.Slot0.kV = ANGLE_CHANGING_CONFIGS.KV;
    config.Slot0.kA = ANGLE_CHANGING_CONFIGS.KA;

    m_request  = new DutyCycleOut(0.0).withUpdateFreqHz(FreqHz);
    velocityVoltage = new VelocityVoltage(0).withSlot(0).withUpdateFreqHz(FreqHz);
    motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0).withUpdateFreqHz(FreqHz); 

    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = -1; /*TODO change to real angle */

    config.MotionMagic.MotionMagicCruiseVelocity = ANGLE_CHANGING_VAR.ANGLE_CHANGING_MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = ANGLE_CHANGING_VAR.ANGLE_CHANGING_MAX_Acceleration;
    config.MotionMagic.MotionMagicJerk = ANGLE_CHANGING_VAR.ANGLE_CHANGING_MAX_JERK;

    config.Feedback.SensorToMechanismRatio = OOM_METER_PER_SPIN;
    
    angleChangingMotor.getConfigurator().apply(config);
    angleChangingMotor.setNeutralMode(NeutralModeValue.Brake);
    /*TODO add brake invert  */
  }
   


  public void setMotorPower(double power){
    angleChangingMotor.setControl(m_request.withOutput(power));
  }

  public void MotionMagic(double position){
    angleChangingMotor.setControl(motionMagicVoltage.withPosition(position));
  }

  public void angleChangingPID(double vel){
    angleChangingMotor.setControl(velocityVoltage.withVelocity(vel));
  }

  public void goToAngle(double angle){
    angleChangingMotor.setPosition(2*A*Math.cos(angle)*OOM_METER_PER_SPIN*ANGLE_CHANGING_GEAR_RATIO);
  }

  public double getC(){
    return angleChangingMotor.getPosition().getValue()/ANGLE_CHANGING_GEAR_RATIO*OOM_SPIN_PER_METER + C_AT_TOP;
  }

  public double getShooterAngle(){
    return Math.acos(-(A*A-B*B-Math.pow(getC(), 2))/(2*B*getC()));
  }

  /**
   * 
   * @return returns if reverse hard limit is closed
   * 
    */
  public boolean isMaxAngle(){
    return angleChangingMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

}
