// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.Constants.*;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {

  
  private TalonFX motor1 = new TalonFX(1);
  private TalonFX motor2 = new TalonFX(4);
  private TalonFX motor3 = new TalonFX(7);
  private TalonFX motor4 = new TalonFX(10);
  public static double num;
  private TalonFXConfiguration config;
  private VelocityVoltage velocityVoltage;
  /** Creates a new TestSubsystem. */
  public TestSubsystem() {
    config = new TalonFXConfiguration();
    config.Slot0.kP = KP;
    //config.Slot0.kS = KS;
    //config.Slot0.kV = KV;
    velocityVoltage = new VelocityVoltage(0);
    config.Voltage.PeakForwardVoltage = 8;
    config.Voltage.PeakReverseVoltage = -8;
    config.MotorOutput.NeutralMode =  NeutralModeValue.Brake;
    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
    motor3.getConfigurator().apply(config);
    motor4.getConfigurator().apply(config);
  }

  //0.638/8.14 = x; 300/63.8


  public void moveWithPid(double speed){
    motor1.setControl(velocityVoltage.withVelocity(speed*GEAR_RATIO/SCOPE));
    motor2.setControl(velocityVoltage.withVelocity(speed*GEAR_RATIO/SCOPE));
    motor3.setControl(velocityVoltage.withVelocity(speed*GEAR_RATIO/SCOPE));
    motor4.setControl(velocityVoltage.withVelocity(speed*GEAR_RATIO/SCOPE));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
  }

  public void setPowers(double power){
    motor1.set(power);
    motor2.set(power);
    motor3.set(power);
    motor4.set(power);
  }
  
  public void setPower(double power){
    motor1.set(power);
  }
  public double getTrueVelocity(){
    return motor1.getVelocity().getValue()*GEAR_RATIO/SCOPE;
  }

  public double getWantedSpeed(){
    return 0.1;
  }

  public void setNum(double num){
    this.num = num;
  }
  public double getNum(){
    return num;
  }

  public double getVelocity(){
    return 1;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("true velocity", this::getTrueVelocity, null);
      builder.addDoubleProperty("wanted Velocity", this::getWantedSpeed, null);
      builder.addDoubleProperty("Speed", this::getNum, this::setNum);
  }
}
