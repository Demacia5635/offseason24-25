// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Subsystems.AngleChanger;
import static frc.robot.Shooter.ShooterConstants.*;

public class Calibration extends Command {
  /** Creates a new Calibrition. */
  private AngleChanger angleChanger;
  private int finishedState = 0;
  public Calibration(AngleChanger angleChanger) {
    this.angleChanger = angleChanger;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(angleChanger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleChanger.angleChangingPID(UP_SPEED_CALIBRATION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(angleChanger.isMaxAngle()){
      angleChanger.angleChangingPID(-DOWN_SPEED_CALIBRATION);
      finishedState = 1;
    }
    if(!angleChanger.isMaxAngle() && finishedState == 1){
      angleChanger.setAngle(TOP_ANGLE);
      finishedState = 2;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleChanger.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (finishedState == 2);
  }
}
