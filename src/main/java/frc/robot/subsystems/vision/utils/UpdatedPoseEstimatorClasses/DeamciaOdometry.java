package frc.robot.subsystems.vision.utils.UpdatedPoseEstimatorClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.WheelPositions;
import frc.robot.subsystems.chassis.ChassisConstants;

public class DeamciaOdometry {
    private Pose2d pose;

    private Kinematics kinematics;
    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;
    private WheelPositions m_previousWheelPositions;
    
    public DeamciaOdometry(Rotation2d gyroAngle, WheelPositions wheelPositions, Pose2d initialPoseMeters) {
        this.pose = initialPoseMeters;
        this.kinematics = ChassisConstants.KINEMATICS_CORRECTED;

        m_gyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = initialPoseMeters.getRotation();
        m_previousWheelPositions = wheelPositions.copy();
    }

    public void resetPosition(Rotation2d gyroAngle, WheelPositions wheelPositions, Pose2d poseMeters) {
        pose = poseMeters;
        m_previousAngle = pose.getRotation();
        m_gyroOffset = pose.getRotation().minus(gyroAngle);
        m_previousWheelPositions = wheelPositions.copy();
    }

    public Pose2d getPose(){
        return pose;
    }

    public Pose2d update(Rotation2d gyroAngle, WheelPositions wheelPositions) {
        Rotation2d angle = gyroAngle.plus(m_gyroOffset);
    
        Twist2d twist = kinematics.toTwist2d(m_previousWheelPositions, wheelPositions);
        twist.dtheta = angle.minus(m_previousAngle).getRadians();
    
        Pose2d newPose = pose.exp(twist);
    
        m_previousWheelPositions = wheelPositions.copy();
        m_previousAngle = angle;
        pose = new Pose2d(newPose.getTranslation(), angle);
    
        return pose;
      }

    
}
