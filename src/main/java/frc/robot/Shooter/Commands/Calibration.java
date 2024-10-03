// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Subsystems.AngleChanger;
import static frc.robot.Shooter.ShooterConstants.*;

public class Calibration extends Command {
  /** Creates a new Calibrition. */
  private AngleChanger angleChanging;
  private int finishedState = 0;
  public Calibration() {
    angleChanging = new AngleChanger();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(angleChanging);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleChanging.angleChangingPID(UP_SPEED_CALIBRATION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(angleChanging.isMaxAngle()){
      angleChanging.angleChangingPID(-DOWN_SPEED_CALIBRATION);
      finishedState = 1;
    }
    if(!angleChanging.isMaxAngle() && finishedState == 1){
      angleChanging.goToAngle(TOP_ANGLE);
      finishedState = 2;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleChanging.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (finishedState == 2);
  }
}
