package frc.robot.utils.ImageProsesing;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TagPoseCalc {
    private double height;
    private double x_offset;
    private double y_offset;
    private double tagYaw;
    private double tagPitch;
    private double id;
    private double sumdegry;
    private Pose2d point;
    private Rotation2d giroYaw;
    private boolean isRed;
    public boolean isIedMostly;


    public TagPoseCalc(double tagYaw, double tagPitch, double x_offset, double y_offset, double id,Rotation2d giroYaw, boolean isRed) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.id = id;
        this.tagYaw = tagYaw;
        this.tagPitch = tagPitch;
        this.giroYaw = giroYaw;
        this.height = ConstantsVision.HEIGHT_MAP.get(id);
        this.isRed = isRed;
    }
    public void updatePosValues(double tagYaw, double tagPitch, double x_offset, double y_offset, double id,Rotation2d giroYaw, boolean isRed) {
        this.x_offset = x_offset;
        this.y_offset = y_offset;
        this.id = id;
        this.tagYaw = tagYaw;
        this.tagPitch = tagPitch;
        this.giroYaw = giroYaw;
        this.height = ConstantsVision.HEIGHT_MAP.get(id);
        this.isRed = isRed;
    }


    

    // Calculate distance FROM CAMERA TO TAG
    public double GetDistFromCamera() {
        sumdegry = tagPitch + ConstantsVision.TagLimelightAngle;
        
        double dist = (Math.abs(height - ConstantsVision.TagLimelightHight)) / (Math.tan(Math.toRadians(sumdegry)));
        System.out.println(dist);
        System.out.println("dist" + dist);

        return (dist);
    }




    // Get object identifier (note or AprilTag)
    public String translateIdToHashmap() {
        return id == 0 ? "note" : ("tag_" + id);
        
    }

    // Calculate vector Camera to tag
    public Translation2d getTagToRobot() {

        Translation2d cameraToTag = new Translation2d(GetDistFromCamera(), 
            Rotation2d.fromDegrees(tagYaw)).rotateBy(isRed ? giroYaw : giroYaw.unaryMinus());
        Translation2d robotToCamera = new Translation2d(x_offset, y_offset).rotateBy(isRed ? giroYaw : giroYaw.unaryMinus());
        Translation2d robotToTag = cameraToTag.plus(robotToCamera);
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
        Translation2d originToTag = ConstantsVision.CARTESIANVECTORS_MAP.get(this.translateIdToHashmap());
        if(originToTag != null){
            Translation2d tagToRobot = getTagToRobot();
            originToRobot = originToTag.plus(tagToRobot);

            point = new Pose2d(originToRobot,isRed ? giroYaw : giroYaw.unaryMinus());
            return point;
        }
        return new Pose2d();
    }

}