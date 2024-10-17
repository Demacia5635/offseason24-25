package frc.robot.vision.utils;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.vision.utils.ConstantsVision.*;
public class TagPoseCalc {
    private double height;
    private double x_offset;
    private double y_offset;
    private double tagYaw;
    private double tagPitch;
    private double id;
    private double sumdegry;
    private Pose2d point;
    private Rotation2d gyroYaw;
    private boolean isRed;
    public boolean isIedMostly;


    public TagPoseCalc(double tagYaw, double tagPitch, double x_offset, double y_offset, double id,Rotation2d gyroYaw, boolean isRed) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.id = id;
        this.tagYaw = tagYaw;
        this.tagPitch = tagPitch;
        this.gyroYaw = gyroYaw;
        this.height = HEIGHT_MAP.get(id);
        this.isRed = isRed;
    }
    public void updatePosValues(double tagYaw, double tagPitch, double x_offset, double y_offset, double id,Rotation2d gyroYaw, boolean isRed) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.id = id;
        this.tagYaw = tagYaw;
        this.tagPitch = tagPitch;
        this.gyroYaw = gyroYaw;
        this.height = HEIGHT_MAP.get(id);
        this.isRed = isRed;
    }


    

    // Calculate distance FROM CAMERA TO TAG
    public double GetDistFromCamera() {
        sumdegry = tagPitch + TagLimelightAngle;
        
        double dist = (Math.abs(height - TagLimelightHeight)) / (Math.tan(Math.toRadians(sumdegry)));
       

        return (dist);
    }




    // Get object identifier (note or AprilTag)
    public String translateIdToHashmap() {
        return id == 0 ? "note" : ("tag_" + id);
        
    }

    public Translation2d getTagToRobot() {

        Translation2d cameraToTag = new Translation2d(GetDistFromCamera(), 
            Rotation2d.fromDegrees(tagYaw)).rotateBy(gyroYaw);
        Translation2d robotToCamera = new Translation2d(x_offset, y_offset).rotateBy(gyroYaw);
        Translation2d robotToTag = robotToCamera.plus(cameraToTag);

        return robotToTag;
    }

    // public Translation2d getTagToRobot() {
    //     Translation2d robotToCamera = new Translation2d(x_offset, y_offset);
    //     Translation2d cameraToTag = new Translation2d(GetDistFromCamera(), Rotation2d.fromDegrees(tx));
    //     Translation2d tagToRobot = robotToCamera.plus(cameraToTag);
    //     return tagToRobot;
    // }


    //get position of robot on the field origin is the point (0,0)!!!!
    public Pose2d calculatePose() {
        Translation2d originToRobot;
        Translation2d originToTag = CARTESIANVECTORS_MAP.get(this.translateIdToHashmap());
        if(originToTag != null){
            Translation2d tagToRobot = getTagToRobot();
            originToRobot = originToTag.plus(tagToRobot);

            point = new Pose2d(originToRobot,isRed ? gyroYaw : gyroYaw.unaryMinus());
            return point;
        }
        return new Pose2d();
    }

}