// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SubSystem;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Cancoder;
import frc.robot.utils.Pigeon;

public class TestSubSytem extends SubsystemBase {
  /** Creates a new TestSubSytem. */
  private Cancoder cancoder;
  private Pigeon pigeon;
  public TestSubSytem(int CancoderID, String CancoderCanBus, int PigeonID, String PigeonCanBus) {
    cancoder = new Cancoder(CancoderID, CancoderCanBus);
    pigeon = new Pigeon(PigeonID, PigeonCanBus);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("getNonAbsRotation2d", cancoder.getNonAbsRotation2d().getRotations());
    SmartDashboard.putNumber("getAbsRotation2d", cancoder.getAbsRotation2d().getRotations());
    SmartDashboard.putNumber("getVelocityRotation2d", cancoder.getVelocityRotation2dPerSec().getRotations());
    SmartDashboard.putNumber("getPigeonAngleDegree", pigeon.getPigeonAngleDegree());
    SmartDashboard.putNumber("getPigeonPitchDegree", pigeon.getPigeonPitchDegree());
    SmartDashboard.putNumber("getPigeonRollDegree", pigeon.getPigeonRollDegree());
    SmartDashboard.putNumber("getXVelocityDegreePerSecond", pigeon.getXVelocityDegreePerSecond());
    SmartDashboard.putNumber("getYVelocityDegreePerSecond", pigeon.getYVelocityDegreePerSecond());
    SmartDashboard.putNumber("getZVelocityDegreePerSecond", pigeon.getZVelocityDegreePerSecond());
  }
  public void resetPigeon(){
    pigeon.resetPigeon();
  }
  public void setOffset(double offset){
    cancoder.setOffset(Rotation2d.fromRotations(offset));
  }
  public void setCanCoderClockwise(Boolean boolDirection){
    cancoder.setCanCoderClockwise(boolDirection);;
  }
}
