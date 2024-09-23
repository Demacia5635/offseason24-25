// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake.Subsystem;
import static frc.robot.Intake.IntakeConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase {
/** Creates a new intake. */
  private TalonFX intakeMotorDown;
  private TalonFX intakeMotorUp;
  public AnalogInput analogInput;
  public Intake() {
    intakeMotorDown = new TalonFX(INTAKE_MOTOR_UP_ID,CANBUS);
    intakeMotorUp = new TalonFX(INTAKE_MOTOR_DOWN_ID,CANBUS);
  }

  @Override
  public void periodic() {

  }
  public void setPower(double power){
    intakeMotorDown.set(power);
    intakeMotorUp.set(power);
  }

  public boolean isNote(){
    return analogInput.getVoltage() > NOTE_VOLTEGE;

  }
  public boolean isAmperHighMotorDown(){
    return intakeMotorDown.getSupplyCurrent().getValue() >= NOTE_CURRENT;
  }
  public boolean isAmperHighMotorUp(){
    return intakeMotorUp.getSupplyCurrent().getValue() >= NOTE_CURRENT;
  }

  public void setPowerMotorUp(double power) {
    intakeMotorUp.set(power);
    
  }
}
