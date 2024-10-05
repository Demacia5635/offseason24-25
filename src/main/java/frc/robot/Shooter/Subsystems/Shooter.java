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

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Shooter.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  
  private TalonFX motorUp;
  private TalonFX motorDown;
  private TalonSRX motorFeeding;
  /**remove when mergin to master */
  public static boolean tempIRSensor = false;

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

    m_request = new DutyCycleOut(0.0).withUpdateFreqHz(SHOOTER_CONFIGS.SHOOTER_FreqHz);
    velocityVoltage = new VelocityVoltage(0).withSlot(0).withUpdateFreqHz(SHOOTER_CONFIGS.SHOOTER_FreqHz);
    configShooting = new TalonFXConfiguration();
    
    configShooting.Slot0.kP = SHOOTER_PID_FF.KP;
    configShooting.Slot0.kI = SHOOTER_PID_FF.KI;
    configShooting.Slot0.kD = SHOOTER_PID_FF.KD;
    configShooting.Slot0.kS = SHOOTER_PID_FF.KS;
    configShooting.Slot0.kV = SHOOTER_PID_FF.KV;
    
    configShooting.MotorOutput.NeutralMode = SHOOTER_CONFIGS.IS_SHOOTING_MOTORS_BRAKE
    ? NeutralModeValue.Brake
    : NeutralModeValue.Coast;

    configDown = configShooting;
    configUp = configShooting;    

    configUp.MotorOutput.Inverted = SHOOTER_CONFIGS.IS_UP_MOTOR_INVERT ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    configDown.MotorOutput.Inverted = SHOOTER_CONFIGS.IS_DOWN_MOTOR_INVERT ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motorUp.getConfigurator().apply(configUp);
    motorDown.getConfigurator().apply(configDown);
    
    motorFeeding.configFactoryDefault();
    motorFeeding.setInverted(SHOOTER_CONFIGS.IS_FEEDING_MOTOR_INVERT);
    motorFeeding.setNeutralMode(SHOOTER_CONFIGS.IS_FEEDING_MOTOR_BRAKE ? NeutralMode.Brake : NeutralMode.Coast);

    SmartDashboard.putData(this);
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
    motorUp.setControl(velocityVoltage.withVelocity(upVel).withFeedForward(getFF(getUpMotorVel())));
    motorDown.setControl(velocityVoltage.withVelocity(downVel).withFeedForward(getFF(getUpMotorVel())));
  }

  public double getFF(double vel) {
    return SHOOTER_PID_FF.KS * Math.signum(vel) + 
    SHOOTER_PID_FF.KV * vel + 
    SHOOTER_PID_FF.KV2 * Math.pow(vel, 2);
  } 

  public void setShootingNeutralMode(boolean isBrake){
    configUp.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    configDown.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    motorUp.getConfigurator().apply(configUp);
    motorDown.getConfigurator().apply(configDown);
  }

  public void setFeedingNeutralMode(boolean isBrake){
    motorFeeding.setNeutralMode(isBrake ? NeutralMode.Brake : NeutralMode.Coast);
  }


  @Override
  public void initSendable(SendableBuilder builder) {
      SmartDashboard.putData("change shooting brake", new InstantCommand(()-> 
      setShootingNeutralMode(configUp.MotorOutput.NeutralMode==NeutralModeValue.Brake ? false : true)
       , this).ignoringDisable(true));
       SmartDashboard.putData("change feeding brake", new InstantCommand(()-> 
      setFeedingNeutralMode(configUp.MotorOutput.NeutralMode==NeutralModeValue.Brake ? false : true)
       , this).ignoringDisable(true));
      SmartDashboard.putNumber("upMotorVel", getUpMotorVel());
      SmartDashboard.putNumber("downMotorVel", getDownMotorVel());
  }
}
