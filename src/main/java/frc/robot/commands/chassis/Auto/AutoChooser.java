
package frc.robot.commands.chassis.Auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.PathFollow.Util.pathPoint;
import static frc.robot.subsystems.chassis.ChassisConstants.*;

public class AutoChooser {
    Translation2d[] zoneTop = {noteTop, noteMid, note1, note2};
    Translation2d[] zoneMid = {noteMid, noteBottom, note2, note3, note4};
    Translation2d[] zoneBottom = {noteBottom, note4, note5};
    Translation2d[] currentZone = {};

    boolean isRed = false;
    int direction = 0; //based on alliance

    Translation2d centerOfStage = new Translation2d();

    private SendableChooser<Translation2d> firstNote;
    private SendableChooser<Translation2d> secondNote;
    private SendableChooser<Translation2d> thirdNote;
    private SendableChooser<Translation2d> fourthNote;
    private SendableChooser<Translation2d> fifthhNote;
    private SendableChooser<Translation2d[]> startingZone;


    
    List<pathPoint> points = new ArrayList<pathPoint>();

    
    public AutoChooser(boolean isRed){
        this.isRed = isRed;
        direction = isRed ? 1 : -1;

        startingZone.setDefaultOption("null", null);
        startingZone.addOption("TOP", zoneTop);
        startingZone.addOption("MIDDLE", zoneMid);
        startingZone.addOption("BOTTOM", zoneBottom);

    }

    public void setZone(){
        if(startingZone.getSelected() == null) System.out.println("Zone not selected");
        else currentZone = (Translation2d[]) startingZone.getSelected();
    }

    

    private pathPoint[] goThroughStage(Translation2d startingPos, Translation2d finalPos){
        List<pathPoint> points = new ArrayList<pathPoint>();
        if((finalPos.getX() - startingPos.getX()) * direction > 0)
        {
           
            if(startingPos.getY() > centerOfStage.getY()) 
                points.add(new pathPoint(12.283, 3.206, Rotation2d.fromDegrees(0), 0.3, false));
            else points.add(new pathPoint(12.195, 5.023, Rotation2d.fromDegrees(0), 0.3, false)); 
            points.add(new pathPoint(11.734, 4.247, Rotation2d.fromDegrees(180), 0.3, false)); 
            points.add(new pathPoint(10.337, 4.231, Rotation2d.fromDegrees(180), 0.3, false)); 
        }
        else {
            points.add(new pathPoint(10.337, 4.231, Rotation2d.fromDegrees(180), 0.3, false));
            points.add(new pathPoint(11.734, 4.247, Rotation2d.fromDegrees(180), 0.3, false)); 
            if(finalPos.getY() > centerOfStage.getY()){
                points.add(new pathPoint(12.195, 5.023, Rotation2d.fromDegrees(0), 0.3, false)); 
            }
            else points.add(new pathPoint(12.283, 3.206, Rotation2d.fromDegrees(0), 0.3, false));
        }
        return (pathPoint[]) points.toArray();
    }


    public void showLegalPoints(Translation2d[] zone){
        if(zone == zoneTop){
            firstNote.setDefaultOption("none", new Translation2d());
            secondNote.setDefaultOption("none", new Translation2d());
            thirdNote.setDefaultOption("none", new Translation2d());
            fourthNote.setDefaultOption("none", new Translation2d());
            firstNote.addOption("Note Top", noteTop);
            firstNote.addOption("Note Mid", noteMid);
            firstNote.addOption("Note 1", note1);
            firstNote.addOption("Note 2", note2);
            secondNote.addOption("Note Top", noteTop);
            secondNote.addOption("Note Mid", noteMid);
            secondNote.addOption("Note 1", note1);
            secondNote.addOption("Note 2", note2);
            thirdNote.addOption("Note Top", noteTop);
            thirdNote.addOption("Note Mid", noteMid);
            thirdNote.addOption("Note 1", note1);
            thirdNote.addOption("Note 2", note2);
            fourthNote.addOption("Note Top", noteTop);
            fourthNote.addOption("Note Mid", noteMid);
            fourthNote.addOption("Note 1", note1);
            fourthNote.addOption("Note 2", note2);
            
            
            
        }
        else if(zone == zoneMid){
            firstNote.addOption(null, note1);
        }
    }

}
