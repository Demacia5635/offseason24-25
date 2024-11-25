package frc.robot.dayC.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.subsystems.Chassis;

public class MoveY extends Command {

  Chassis chassis;
  double dis;
  double startingPos;

  /**
   * move the chassis relative angle
   * @param chassis
   * @param dis in meter
   */
  public MoveY(Chassis chassis, double dis) {
    this.chassis = chassis;
    this.dis = dis;
  }

  @Override
  public void initialize() {
    startingPos = chassis.getPoseY();
  }

  @Override
  public void execute() {
    chassis.setVelocitiesRobotRel(new ChassisSpeeds(0, dis >= 0 ? -1 : 1, 0));
  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(chassis.getPoseY() - startingPos) >= Math.abs(dis);
  }
}
