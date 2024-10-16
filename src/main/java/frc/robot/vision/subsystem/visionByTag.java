// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.ConstantsVision;
import frc.robot.vision.utils.PositionFilter;
import frc.robot.vision.utils.TagPoseCalc;

public class visionByTag extends SubsystemBase {
  /** Creates a new vision. */
   // NetworkTable for Limelight communication
  private NetworkTable table;
  
  // Limelight data
  private double x_offset;
  private double y_offset;
  private double tagYaw;
  private double tagPitch;
  private double id;
  private Pigeon2 gyro;

  // Pose and distance calculation utilities
  private TagPoseCalc Pose;

  //pose of robot in field 
  Field2d field;

  private PositionFilter positionFilter;

  public visionByTag(Pigeon2 gyro) {
    this.gyro = gyro;

    // Initialize Field2d for visualization
    
    this.x_offset = ConstantsVision.TagLimelightXOfset;
    this.y_offset = ConstantsVision.TagLimelightYOfset;    

    // Get the Limelight NetworkTable
    table = NetworkTableInstance.getDefault().getTable(ConstantsVision.TagTable);
    Pose = new TagPoseCalc(tagYaw, tagPitch, x_offset, y_offset, id,Rotation2d.fromDegrees(gyro.getAngle()), false);

    field = new Field2d();
    
    SmartDashboard.putData(this);

    positionFilter = new PositionFilter(5, 10, 2.0, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Fetch Limelight data
    tagYaw = table.getEntry("tx").getDouble(0);
    tagPitch = table.getEntry("ty").getDouble(0);
    id = table.getEntry("tid").getDouble(0);
    tagYaw *=-1;
    // Update Pose
    Pose.updatePosValues(tagYaw, tagPitch, x_offset, y_offset, id, Rotation2d.fromDegrees(gyro.getAngle()), false);

    // Get the raw robot pose
    Pose2d rawPose = getRobotPose();

    // Apply the filter
    Pose2d filteredPose = positionFilter.update(rawPose, 0.02);

    // Update the field visualization with the filtered pose
    field.setRobotPose(filteredPose);
  }
  /**
   * returns the position of the robot
   * @return pose of robot
   */
  public Pose2d getRobotPose() {
    Pose2d robotPose = Pose.calculatePose();
    return (robotPose != null) ? robotPose : new Pose2d();
}
  /**
   * return the tag the camera sees, if she does not look at tag return -1;
   * @return the id of the tag the camera sees;
   */
  public double getId(){
    return id;
  }
  /**
   * 
   * @return the distens from robot to tag(getNoorm);
   */
  public double getDistToTag(){
    return Pose.getTagToRobot().getNorm();
  }
  /**
   * 
   * @return the angle from robot to tag(getAngle);
   */
  public Rotation2d getAngleTag(){
    return Pose.getTagToRobot().getAngle();
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    SmartDashboard.putData("field-tag", field);
  }

}
