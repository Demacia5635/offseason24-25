// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

/** Add your docs here. */
public class DemaciaOdometry {
    Pose2d curPose;
    Rotation2d gyroOffset;
    private Rotation2d prevAngle;
    private SwerveDriveWheelPositions prevWheelPositions;
    private BuiltInAccelerometer accelerometer;
    private DemaciaKinematics kinematics;

    //How correct the estimation is (50% currently)
    private double wheelCalcWeight = 0.5;
    private double accelCalcWeight = 0.5;

    private final double cycleTime = 0.02;

    public DemaciaOdometry(DemaciaKinematics kinematics, Rotation2d gyroAngle, SwerveDriveWheelPositions wheelPositions, Pose2d initPose){
        this.curPose = initPose;
        this.gyroOffset = curPose.getRotation().minus(gyroAngle);
        this.prevAngle = gyroAngle;
        this.prevWheelPositions = wheelPositions;
        this.kinematics = kinematics;
        this.accelerometer = new BuiltInAccelerometer();
    }
    
    public void resetPosition(Rotation2d gyroAngle, SwerveDriveWheelPositions wheelPos, Pose2d poseToReset){
        curPose = poseToReset;
        prevAngle = curPose.getRotation();
        gyroOffset = curPose.getRotation().minus(prevAngle);
        prevWheelPositions = wheelPos;
    }

    public Pose2d getCurPose(){
        return curPose;
    }

    private double gForceToMS2(double gForce){
        return gForce * 9.80665;
    }
    private double ms2ToGForce(double accel){
        return accel / 9.80665;
    }


    //calculates total Pose using accelerometer and wheels
    public Pose2d calcEstimatedPose(Rotation2d gyroAngle, SwerveDriveWheelPositions currentWheelPositions){

        Pose2d estimatedPoseAccel = new Pose2d(curPose.getTranslation().plus(calcChassisDiffAccel()), gyroAngle.minus(gyroOffset));
        Pose2d estimatedPoseWheels = estimateWheels(gyroAngle, currentWheelPositions);
        curPose = fuse(estimatedPoseWheels, estimatedPoseAccel);

        return curPose;
        
    }

    //fusing 2 Pose estimations using the weight parameter for each estimation
    public Pose2d fuse(Pose2d estimatedWheels, Pose2d estimatedAccel){
        double x = (estimatedWheels.getX() * wheelCalcWeight) + (estimatedAccel.getX() * accelCalcWeight);
        double y = (estimatedWheels.getY() * wheelCalcWeight) + (estimatedAccel.getY() * accelCalcWeight);
        Rotation2d rotation = (estimatedWheels.getRotation().times(wheelCalcWeight)).plus(estimatedAccel.getRotation().times(accelCalcWeight));

        return new Pose2d(x, y, rotation);
    }

    
     

    //calcs
    public Pose2d estimateWheels(Rotation2d gyroAngle, SwerveDriveWheelPositions currentWheelPositions){

        Rotation2d angle = gyroAngle.plus(gyroOffset);
        Twist2d twist = kinematics.toTwist2d(prevWheelPositions, currentWheelPositions);
        twist.dtheta = angle.minus(prevAngle).getRadians();
        Pose2d newPose = curPose.exp(twist);
        prevWheelPositions = currentWheelPositions.copy();
        prevAngle = angle;
        return new Pose2d(newPose.getTranslation(), angle);
         
    }
    



    private Translation2d calcChassisDiffAccel(){
        double xDiff = 0.5 * gForceToMS2(accelerometer.getX()) * cycleTime * cycleTime;
        double yDiff = 0.5 * gForceToMS2(accelerometer.getY()) * cycleTime * cycleTime;

        return new Translation2d(xDiff, yDiff);
    }
}
