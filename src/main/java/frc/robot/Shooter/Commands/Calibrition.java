// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Subsystems.AngleChanging;
import static frc.robot.Shooter.ShooterConstants.*;

public class Calibrition extends Command {
  /** Creates a new Calibrition. */
  private AngleChanging angleChanging;
  private int isFinished = 0;
  public Calibrition() {
    angleChanging = new AngleChanging();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(angleChanging);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleChanging.anglerPID(UP_SPEED_CALIBRATION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(angleChanging.isTopAngle()){
      angleChanging.anglerPID(-DOWN_SPEED_CALIBRATION);
      isFinished = 1;
    }
    if(!angleChanging.isTopAngle() && isFinished == 1){
      isFinished = 2;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleChanging.setAngleChangingMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isFinished == 2);
  }
}
