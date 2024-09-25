package frc.robot.utils.ImageProsesing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class NotePoseCalc {
    private double height;
    private double x_offset;
    private double y_offset;
    private double tx;
    private double ty;
    private double id;
    private double sumdegry;
    private Pose2d pose;
    private Rotation2d giroYaw;
    public boolean isIedMostly;



    public NotePoseCalc(double tx, double ty, double x_offset, double y_offset,Pose2d pose) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.tx = tx;
        this.ty = ty;
        this.pose = pose;
        this.giroYaw = pose.getRotation();
        this.height = Constants.HEIGHT_MAP.get(id);
    }

    

    // Calculate distance FROM CAMERA TO TAG
    public double GetDistFromCamera() {
        sumdegry = ty + Constants.NoteLimelightAngle;
        sumdegry = Math.toRadians(sumdegry);
        return ((Math.abs(height - Constants.NoteLimelightHight)) * (Math.tan(sumdegry)));
    }




    // Calculate vector Camera to tag
    public Translation2d getRobotToNote() {
        Translation2d cameraToNote = new Translation2d(GetDistFromCamera(), Rotation2d.fromDegrees(tx)).rotateBy(giroYaw);

        
        Translation2d robotToCamera = new Translation2d(x_offset, y_offset*Math.signum(tx)).rotateBy(giroYaw);
        Translation2d robotToNote = cameraToNote.plus(robotToCamera);
        return robotToNote;
    }

    //get position of robot on the field origin is the pose (0,0)!!!!
    public Pose2d calculatePose() {
        Translation2d originToNote;
        Translation2d originToRobot = pose.getTranslation();
        if(tx != 0 && ty != 0 ){
            Translation2d robotToNote = getRobotToNote();
            originToNote = originToRobot.plus(robotToNote);
            

            pose = new Pose2d(originToNote,new Rotation2d());

        }
        return pose;
    }

}