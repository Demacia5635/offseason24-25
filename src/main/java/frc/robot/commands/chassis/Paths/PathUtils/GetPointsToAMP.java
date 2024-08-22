
package frc.robot.commands.chassis.Paths.PathUtils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathFollow.Util.CurrentPosOnField.Zone;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.chassis.Chassis;

public class GetPointsToAMP extends Command {
  Chassis chassis;
  List<pathPoint> points = new ArrayList<pathPoint>();
  /**
   * 
   * @param chassis
   * @param lastPoint
   */
  public GetPointsToAMP(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    points.add(new pathPoint(chassis.getPose().getX(), chassis.getPose().getY(),
     chassis.getAngle(), 0.3, false));
     
    
    Zone zone = Zone.getZone(chassis.getPose().getTranslation());
    // System.out.println("ZONE: " + zone);
    switch (zone) {
      case STAGE:
        points.add(new pathPoint(11.811, 3.203, chassis.getAngle(), 0.15, false));
       
        break;
      case SOURCE:
        points.add(new pathPoint(4.38, 5.105, chassis.getAngle(), 0.25, false));
        points.add(new pathPoint(6.216, 3.763, chassis.getAngle(), 0.15, false));
        points.add(new pathPoint(11.777, 4.168, chassis.getAngle(), 0.1, false));
        points.add(new pathPoint(13.076, 2.209, chassis.getAngle(), 0.1, false));



        break;
      case SPEKAER:
      points.add(new pathPoint(14.585, 1.611, chassis.getAngle(), 0.5, false));

        break;
      default:
      //blue alliance on the right
      //case for robot is in X center-right Y top 
        if(chassis.getPose().getY() >= 5.75 && chassis.getPose().getX() >= 8.353){
          points.add(new pathPoint(14.735, 6, chassis.getAngle(), 0.45, false));
        }
        //case for robot is in X center-right Y center
        else if(chassis.getPose().getY() < 5.75 && chassis.getPose().getY() >= 2.387 &&
         chassis.getPose().getX() >= 8.353){
          points.add(new pathPoint(11.525, 4.41, chassis.getAngle(), 0.45, false));
          points.add(new pathPoint(12.425, 3.607, chassis.getAngle(), 0.1, false));
        //case for robot is in bottom (no need to add more points then alreay made first and last point) 
        } else if(chassis.getPose().getY() < 2.387){

        } 
        //case for robot is in X center-left / left Y top
        else if(chassis.getPose().getY() >= 5.75 && chassis.getPose().getX() <= 8.353){
          points.add(new pathPoint(9.574, 4.366, chassis.getAngle(), 0.25, false));
          points.add(new pathPoint(11.72, 4.395, chassis.getAngle(), 0.5, false));
          points.add(new pathPoint(13.0151, 2.7, chassis.getAngle(), 0.15, false));
        }
        //case for robot is in X center-left Y center
        else{
          points.add(new pathPoint(11.72, 4.395, chassis.getAngle(), 0.5, false));
          points.add(new pathPoint(13.0151, 2.7, chassis.getAngle(), 0.15, false));
        }
      
        break;
    }
    //last point of GoToAMP (Closet point to AMP with correct rotation)
    points.add(new pathPoint(14.321, 0.1, Rotation2d.fromDegrees(-90), 0.25, false));
    Chassis.pointsForPathTeleop = points;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
