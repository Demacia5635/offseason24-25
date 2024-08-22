package frc.robot.commands.chassis.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.SwerveModule;

public class ResetWheelCommand extends Command {
  private final SwerveModule module;

  public ResetWheelCommand(SwerveModule module) {
    this.module = module;
  }

  @Override
  public void execute() {
    module.setAngleByPositionPID(new Rotation2d());
  }

  @Override
  public boolean isFinished() {
    return Math.abs(module.getAngle().getDegrees()) < 3;  
  }
}
