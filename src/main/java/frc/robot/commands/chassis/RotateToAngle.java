// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;
import frc.robot.utils.Trapezoid;

public class RotateToAngle extends Command {

    Chassis chassis;
    Rotation2d angle;
    Trapezoid trapezoid;
    double error;
    double minError = Math.toRadians(1);

  public RotateToAngle(double angle, Chassis chassis) {
    this.angle = Rotation2d.fromDegrees(angle);
    this.chassis = chassis;
    trapezoid = new Trapezoid(ChassisConstants.MAX_OMEGA_VELOCITY, ChassisConstants.MAX_OMEGA_ACCELERATION);
    addRequirements(chassis);
  }

  @Override
  public void execute() {
      Rotation2d current = chassis.getAngle();
      error = angle.minus(current).getRadians();
      var speeds = chassis.getChassisSpeeds();
      double v = trapezoid.calculate(error, speeds.omegaRadiansPerSecond, 0);
      var s = new ChassisSpeeds(0,0,v);
      chassis.setVelocities(s);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < minError;
  }
}
