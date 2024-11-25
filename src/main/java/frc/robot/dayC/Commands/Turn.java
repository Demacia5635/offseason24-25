package frc.robot.dayC.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.subsystems.Chassis;

public class Turn extends Command {

  Chassis chassis;
  double angle;
  double startingAngle;

  /**
   * rotate to angle
   * @param chassis
   * @param angle in radians
   */
  public Turn(Chassis chassis, double angle) {
    this.chassis = chassis;
    this.angle = angle;
  }

  @Override
  public void initialize() {
    startingAngle = chassis.getAngle().getRadians();
  }

  @Override
  public void execute() {
    chassis.setVelocitiesRobotRel(new ChassisSpeeds(0, 0, angle >= 0 ? Math.PI / 4 : -1 * Math.PI / 4));
  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(chassis.getAngle().getRadians() - startingAngle) >= Math.abs(angle);
  }
}
