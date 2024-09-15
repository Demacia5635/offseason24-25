// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Subsystems.Shooter;

public class ShootNote extends Command {
  /** Creates a new ShootNote. */
  private Shooter shooter;
  private double upMotorVelocity;
  private double downMotorVelocity;
  private double feedingPower;
  public ShootNote(double upMotorVelocity, double downMotorVelocity, double feedingPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.upMotorVelocity = upMotorVelocity;
    this.downMotorVelocity = downMotorVelocity;
    this.feedingPower = feedingPower;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setFeedingPower(feedingPower);
    shooter.setUpMotorVelocityPid(upMotorVelocity);
    shooter.setDownMotorVelocityPid(downMotorVelocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFeedingPower(0);
    shooter.setUpMotorPower(0);
    shooter.setDownMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
