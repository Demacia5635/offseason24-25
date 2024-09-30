
package frc.robot.commands.chassis.Paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.shooter.Shooter;



public class GoToAMP1 extends Command {
  
    pathPoint[] toAmp = {
      new pathPoint(new Translation2d(), new Rotation2d()),
      new pathPoint(Field.offset(Field.AMP, 0, 0.6), Rotation2d.fromDegrees(-90))
    };

    Command cmd;

    @Override
    public void initialize() {
      Shooter shooter = RobotContainer.robotContainer.shooter;
        cmd = shooter.getActivateShooterToAmp().raceWith(new PathFollow(toAmp,1).andThen(shooter.getShootCommand()));
        cmd.schedule();
    }

    @Override
    public boolean isFinished() {
        return !cmd.isScheduled();
    }
}
