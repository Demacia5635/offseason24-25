// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Subsystems.AngleChanging;

public class getToAngle extends Command {
  /** Creates a new getToAngle. */
  public AngleChanging angleChanging;
  private enum State{
    AMP,SPEAKER
  }
  private State shooterState = State.AMP;
  public getToAngle() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(angleChanging);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (shooterState) {
      case AMP:
        
        break;
    
      case SPEAKER:

        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
