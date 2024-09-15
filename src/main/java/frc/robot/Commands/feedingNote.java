// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class FeedingNote extends Command {
  /** Creates a new feedingNote. */
  private Shooter shooter;
  private double feedingPower;
  public FeedingNote(Shooter shooter, double feedingPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.feedingPower =  feedingPower;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setFeedingPower(feedingPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFeedingPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isNote();
  }
}
