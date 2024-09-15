// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Shooter.ShooterConstants.*;

public class AngleChanging extends SubsystemBase {
  /** Creates a new AngleChanging. */
  private TalonFX angleChangingMotor;
  private AnalogInput analogInput;

  private TalonFXConfiguration config;
  private DutyCycleOut m_request;
  private VelocityVoltage velocityVoltage;

  private MotionMagicVoltage motionMagicVoltage;
  public AngleChanging() {
    angleChangingMotor = new TalonFX(ANGLE_CHANGING_ID, CANBUS);
    analogInput = new AnalogInput(ANALOG_INPUT_ID);
    config = new TalonFXConfiguration();
    config.Slot0.kP = ANGLE_CHANGING_KP;
    config.Slot0.kI = ANGLE_CHANGING_KI;
    config.Slot0.kD = ANGLE_CHANGING_KD;
    config.Slot0.kS = ANGLE_CHANGING_KS;
    config.Slot0.kV = ANGLE_CHANGING_KV;
    config.Slot0.kA = ANGLE_CHANGING_KA;
    config.Voltage.PeakForwardVoltage = 8;
    config.Voltage.PeakReverseVoltage = -8;
    m_request  = new DutyCycleOut(0.0);
    velocityVoltage = new VelocityVoltage(0);
    motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ANGLE_CHANGING_MAX_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = ANGLE_CHANGING_MAX_Acceleration;
    motionMagicConfigs.MotionMagicJerk = ANGLE_CHANGING_MAX_Jerk;
    angleChangingMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAngleChangingMotorPower(double power){
    angleChangingMotor.setControl(m_request.withOutput(power));
  }

  public void setAngleChangingVelocityMotionMagic(double position){
    angleChangingMotor.setControl(motionMagicVoltage.withPosition(position));

  }

  public void setAngle(Double rotation){
    angleChangingMotor.setPosition(rotation);
  }

  public double getShooterAngle(){
    return angleChangingMotor.getPosition().getValue() * OOM_SPIN_PER_METER / ANGLE_CHANGING_GEAR_RATIO;
  }

  public boolean isTopAngle(){
    return analogInput.getVoltage() > SHOOOTER_VOLTAGE;
  }
}
