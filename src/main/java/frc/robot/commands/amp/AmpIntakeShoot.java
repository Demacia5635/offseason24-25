// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpConstants.Parameters;

public class AmpIntakeShoot extends Command {
  Amp amp;
  double startTime;

  /** Creates a new AmpIntakeShoot. */
  public AmpIntakeShoot(Amp amp) {
    this.amp = amp;
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    amp.setIntakePower(-Parameters.INTAKE_TRANSFER_POWER); // Run motors at transfer speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.setIntakePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - startTime > 1) {
      return true;
    }
    return false;
  }
}
