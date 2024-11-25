package frc.robot.dayC.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.chassis.subsystems.Chassis;

public class MoveX extends Command {

  Chassis chassis;
  double dis;
  double startingPos;

  /**
   * move the chassis relative angle
   * @param chassis
   * @param dis in meter
   */
  public MoveX(Chassis chassis, double dis) {
    this.chassis = chassis;
    this.dis = dis;
  }

  @Override
  public void initialize() {
    startingPos = chassis.getPoseX();
  }

  @Override
  public void execute() {
    chassis.setVelocitiesRobotRel(new ChassisSpeeds(dis >= 0 ? 1 : -1, 0, 0));
  }

  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(chassis.getPoseX() - startingPos) >= Math.abs(dis);
  }
}
