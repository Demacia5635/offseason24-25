// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ModuleS extends SubsystemBase {

  private TalonFX motorOne;
  private TalonFX motorTwo;

  private DutyCycleOut m_Requst = new DutyCycleOut(0.0);
  /** Creates a new moduleS. */
  public ModuleS() {
    motorOne = new TalonFX(1);
    motorTwo = new TalonFX(2);
  }

 

  public void power(double power){
    motorOne.setControl(m_Requst.withOutput(power));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

   public double getVelocity(){
    return motorOne.getVelocity().getValue()/GEAR_RATIO*Scope;
  }
}
