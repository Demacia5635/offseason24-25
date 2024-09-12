// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.*;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {

  
  private TalonFX motor = new TalonFX(1);
  private TalonFXConfiguration config;
  private VelocityVoltage velocityVoltage;
  /** Creates a new TestSubsystem. */
  public TestSubsystem() {
    config = new TalonFXConfiguration();
    config.Slot0.kP = KP;
    config.Slot0.kS = KS;
    config.Slot0.kV = KV;
    config.Voltage.PeakForwardVoltage = 8;
    config.Voltage.PeakReverseVoltage = -8;
    motor.getConfigurator().apply(config);
  }

  //0.638/8.14 = x; 300/63.8

  public void moveWithPid(double speed){
    motor.setControl(velocityVoltage.withVelocity(speed/SCOPE));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(this);
  }
  
  
  public double getTrueVelocity(){
    return motor.getVelocity().getValue()/GEAR_RATIO*SCOPE;
  }

  public double getWantedSpeed(){
    return 0.1;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("true velocity", this::getTrueVelocity, null);
      builder.addDoubleProperty("wanted Velocity", this::getWantedSpeed, null);
  }
}
