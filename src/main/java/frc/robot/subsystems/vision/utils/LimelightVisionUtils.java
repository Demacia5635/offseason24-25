package frc.robot.subsystems.vision.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

/**
 * Utility class for vision
 */
public class LimelightVisionUtils {

    public enum LimeLight {
        LimeLight2,
        LimeLight3
    }
    public static final NetworkTable LIMELIGHT_AMP_TABLE = NetworkTableInstance.getDefault().getTable("limelight-amp");
    public static final NetworkTable LIMELIGHT_SHOOTER_TABLE = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    public static final double MAX_DISTANCE_FOR_LIMELIGHT = 6;

    public static NetworkTable[] LimeLightTables = {LIMELIGHT_AMP_TABLE, LIMELIGHT_SHOOTER_TABLE};


    public static double getIdShooter() {
        return LIMELIGHT_SHOOTER_TABLE.getEntry("tid").getDouble(0);
    }

    public static boolean isInFrontOfShooter() {
        double id = (RobotContainer.robotContainer.isRed()) ? 4 : 7;
        return getIdShooter() == id;
    }
    public static boolean isInFrontOfAMP(){
        double id = (RobotContainer.robotContainer.isRed()) ? 5 : 6;
        return getIdShooter() == id;
         
    }

    
    public static Translation2d getDxDy() {
        boolean isInFront = isInFrontOfShooter();
        double dx;
        double dy;
        double[] array;
        if (isInFront) {
            //array = LIMELIGHT_SHOOTER_TABLE.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
            array = LIMELIGHT_SHOOTER_TABLE.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
            dy = array[0];
            dx = -array[2] - 0.10;
           //System.out.println("dx= " + dx +", dy=" + dy);
            return new Translation2d(dx, dy); 
        }
        return null;
        //cannot be in this distance (out of field)
    }
     
    public static double getTA() {
        Translation2d xy = getDxDy();
        if(xy == null) {
            return 1000;
        }
        double rad = Math.atan(xy.getY()/ xy.getX());
        //System.out.println("rad blue= " +  MathUtil.angleModulus(rad));
        if(RobotContainer.robotContainer.isRed()) {
           // System.out.println("rad red= " + MathUtil.angleModulus(Math.PI + rad));
            return  MathUtil.angleModulus(Math.PI + rad);
        }
        return MathUtil.angleModulus(rad);
        //return (xy ==null) ? new Rotation2d(0) : xy.getAngle();
    }

    public static Rotation2d getTAAmp() {
        Translation2d xy = getDxDyAMP();
        return (xy==null)? new Rotation2d() : new Rotation2d(Math.atan(xy.getY()/ xy.getX()));
        //return (xy ==null) ? new Rotation2d(0) : xy.getAngle();
    }

    public static Translation2d getDxDyAMP() {
        boolean isInFront = isInFrontOfShooter();
        double dx;
        double dy;
        double[] array;
        if (isInFront) {
            array = LIMELIGHT_SHOOTER_TABLE.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
            dy = array[0];
            dx = array[2];
            return new Translation2d(dx, dy); 
        }
        return null;
        //cannot be in this distance (out of field)
    }



    



    /**
     * Gets the pose of the robot from the vision system
     * 
     * @return The pose of the robot from the vision system, and the timestamp of
     *         the measurement, or null if no target is found
     */
    public static Pair<Pose2d, Double> getVisionPose(int camera) {
        return getVisionPose(LimeLightTables[camera]);
    }
    public static Pair<Pose2d, Double> getVisionPose(NetworkTable limelightTable) {
        double timeStamp = Timer.getFPGATimestamp();
        double hasTarget = limelightTable.getEntry("tv").getDouble(0);
        if (hasTarget == 0)
            return null;

        double[] robotPose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        // data
        // [0-X, 1-Y, 2-Z, 3-Roll, 4-Pitch, 5-Yaw, 6-Latecy, 7-Tag Count,8-Tag Span, 9-Distance, 10-Tag Area]
        if (robotPose.length <7)
            return null;

        double latency = robotPose[6]/1000.0;
        Rotation2d robotRotation = Rotation2d.fromDegrees(robotPose[5]);
        Translation2d robotTranslation = new Translation2d(robotPose[0], robotPose[1]);
        double distance =  (robotPose.length >= 9) ? robotPose[9]: 1;
        //double distance =  robotPose[9];

        
        if(distance > MAX_DISTANCE_FOR_LIMELIGHT || distance == 0 || 
            Math.abs(RobotContainer.robotContainer.chassis.getChassisSpeeds().omegaRadiansPerSecond) > 1){
            return null;
        }

//        if(limelightTable.g  etEntry("ti").getDouble(0))) 
        return new Pair<Pose2d, Double>(
                new Pose2d(robotTranslation, robotRotation),
                timeStamp - latency);
    }
}