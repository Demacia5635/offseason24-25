// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class shootNote extends Command {
  /** Creates a new shootNote. */
  private Shooter shooter;
  private double motorUpVelocity;
  private double motorDownVelocity;
  public shootNote(Shooter shooter, double motorUpVelocity, double motorDownVelocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.motorUpVelocity = motorUpVelocity;
    this.motorDownVelocity =  motorDownVelocity;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setUpMotorVelocityPid(motorUpVelocity);
    shooter.setUpMotorVelocityPid(motorDownVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setUpMotorPower(0);
    shooter.setDownMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
