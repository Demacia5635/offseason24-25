package frc.robot.vision.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class NotePoseCalc {
    private double x_offset;
    private double y_offset;
    private double tx;
    private double ty;
    private double id;
    private double anglePitch;
    private Pose2d pose;
    private Rotation2d gyroYaw;
    private boolean isRed;


    public NotePoseCalc(double tx, double ty, double x_offset, double y_offset,Pose2d pose, boolean isRed) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.tx = -tx;
        this.ty = ty;
        this.pose = pose;
        this.gyroYaw = pose.getRotation();
        this.isRed = isRed;
    }
    public void update(double tx, double ty, double x_offset, double y_offset,Pose2d pose, boolean isRed) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.tx = tx;
        this.ty = ty;
        this.pose = pose;
        this.gyroYaw = pose.getRotation();
        this.isRed = isRed;
    }

    

    // Calculate distance FROM CAMERA TO TAG
    public double GetDistFromCamera() {
        anglePitch = (ConstantsVision.NoteLimelightAngle-ty );
        // System.out.println("ang"+ty);
        // System.out.println((ConstantsVision.NoteLimelightHight) * (Math.tan(anglePitch)));
        return ((ConstantsVision.NoteLimelightHight) * (Math.tan(anglePitch)));

    }




    // Calculate vector Camera to tag
    public Translation2d getRobotToNote() {
        Translation2d cameraToNote = new Translation2d(GetDistFromCamera(), Rotation2d.fromDegrees(tx)).rotateBy(isRed ? gyroYaw : gyroYaw.unaryMinus());
        Translation2d robotToCamera = new Translation2d(x_offset, y_offset).rotateBy(isRed ? gyroYaw : gyroYaw.unaryMinus());
        Translation2d robotToNote = cameraToNote.plus(robotToCamera);
        return robotToNote;
    }

    //get position of robot on the field origin is the pose (0,0)!!!!
    public Pose2d calculatePose() {
        Translation2d originToNote;
        
        if(tx != 0 && ty != 0 && pose != null){
            Translation2d originToRobot = pose.getTranslation();
            Translation2d robotToNote = getRobotToNote();
            originToNote = originToRobot.plus(robotToNote);
            

            pose = new Pose2d(originToNote,new Rotation2d());
            return pose;
        }
        return new Pose2d();
    }

}