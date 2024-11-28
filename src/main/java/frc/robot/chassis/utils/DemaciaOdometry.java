package frc.robot.chassis.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.Odometry.DemaciaKinematics;
import frc.robot.chassis.ChassisConstants;
import frc.robot.chassis.subsystems.Chassis;

public class DemaciaOdometry {
    private final double DISTANCE_OFFSET = 100; //meters
    private final double ANGLE_OFFSET = 90; //degers
    private final double MAX_X_CRASH = 3;
    private final double MAX_Y_CRASH = 3;
    private final double MAX_Z_CRASH = -3;
    


    private BuiltInAccelerometer accelerometer;

    private Pose2d pose;
    private final int wheelCount;
    private DemaciaKinematics kinematics;
    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;
    private SwerveDriveWheelPositions m_previousWheelPositions;
    
    public DemaciaOdometry(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d initialPoseMeters) {

        this.accelerometer = new BuiltInAccelerometer();
        
        this.pose = initialPoseMeters;
        this.kinematics = ChassisConstants.KINEMATICS_DEMACIA;
        this.wheelCount = wheelPositions.length;

        m_gyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = initialPoseMeters.getRotation();
        m_previousWheelPositions = new SwerveDriveWheelPositions(wheelPositions);
    }

    public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
        pose = poseMeters;
        m_previousAngle = pose.getRotation();
        m_gyroOffset = pose.getRotation().minus(gyroAngle);
        m_previousWheelPositions = new SwerveDriveWheelPositions(wheelPositions);
    }

    public Pose2d getPose(){
        return pose;
    }


/*  public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        if(hasCrashed()){
            return pose;
        }

        Rotation2d angle = gyroAngle.plus(m_gyroOffset);
        SwerveDriveWheelPositions cur = new SwerveDriveWheelPositions(wheelPositions);
        
        
        SwerveModulePosition[] curModulePos = cur.positions;
        boolean[] indexs = new boolean[wheelCount];
        for(int i = 0; i < indexs.length; i++){
            indexs[i] = false;
        }
        for(int i = 0; i < curModulePos.length; i++){
            indexs[i] = isSlipped(curModulePos[i]);
        }
        int correctModulesCount = 0;
        for(boolean bool : indexs){
            if(!bool) correctModulesCount++;
        }

        SwerveModulePosition[] modulesAfterCheckArr = new SwerveModulePosition[correctModulesCount];
        SwerveModulePosition[] prevAfterCheck = new SwerveModulePosition[correctModulesCount];
        for(int i =0; i < correctModulesCount; i++){
            if(indexs[i]) modulesAfterCheckArr[i] = cur.positions[i]; prevAfterCheck[i] = m_previousWheelPositions.positions[i];
        }

        SwerveDriveWheelPositions curModulesAfterCheck = new SwerveDriveWheelPositions(modulesAfterCheckArr);
        SwerveDriveWheelPositions prevModulesAfterCheck = new SwerveDriveWheelPositions(prevAfterCheck);
        
        
        Twist2d twist = kinematics.toTwist2d(prevModulesAfterCheck, curModulesAfterCheck);
        twist.dtheta = angle.minus(m_previousAngle).getRadians();


        Pose2d newPose = pose.exp(twist);
        m_previousWheelPositions = cur;
        m_previousAngle = angle;
        pose = new Pose2d(newPose.getTranslation(), angle);
        

        return pose;
    }
*/


    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        var angle = gyroAngle.plus(m_gyroOffset);
        SwerveDriveWheelPositions cur = new SwerveDriveWheelPositions(wheelPositions);
        


       
        Twist2d twist = kinematics.toTwist2d(m_previousWheelPositions, cur);
        twist.dtheta = angle.minus(m_previousAngle).getRadians();


        Pose2d newPose = pose.exp(twist);
        
        m_previousWheelPositions = cur;
        m_previousAngle = angle;
        pose = new Pose2d(newPose.getTranslation(), angle);
        return pose;
  }

    public Pose2d updateWithVision(Pose2d mergedPose, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions){
        var angle = gyroAngle.plus(m_gyroOffset);

        SwerveDriveWheelPositions cur = new SwerveDriveWheelPositions(wheelPositions);
            
        Twist2d twist = kinematics.toTwist2d(m_previousWheelPositions, cur);
        twist.dtheta = angle.minus(m_previousAngle).getRadians();


        Pose2d newPose = mergedPose.exp(twist);
        m_previousWheelPositions = cur;
        m_previousAngle = angle;
        
        return new Pose2d(newPose.getTranslation(), angle);
        
    }



    private boolean isSlipped(SwerveModulePosition modulePos){
        double maxDistance = Chassis.targetVelocity * 0.02;
        double minDistance = Chassis.currentVelocity * 0.02;
        return modulePos.distanceMeters >= minDistance && modulePos.distanceMeters <= maxDistance; 
    }

    private boolean hasCrashed(){
        return Math.abs(accelerometer.getX()) > MAX_X_CRASH || Math.abs(accelerometer.getY()) > MAX_Y_CRASH || Math.abs(accelerometer.getY()) > MAX_Z_CRASH; 

    }
    

}