// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.vision.utils.ConstantsVision;
import frc.robot.vision.utils.TagPoseCalc;

public class VisionByTag extends SubsystemBase {
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
  NetworkTableEntry tvEntry;
  NetworkTableEntry txEntry;
  NetworkTableEntry tyEntry;
  NetworkTableEntry tidEntry;
  


  public VisionByTag(Pigeon2 gyro) {
    this.gyro = gyro;

    // Initialize Field2d for visualization
    
    this.x_offset = ConstantsVision.TagLimelightXOfset;
    this.y_offset = ConstantsVision.TagLimelightYOfset;    

    // Get the Limelight NetworkTable
    table = NetworkTableInstance.getDefault().getTable(ConstantsVision.TagTable);
    tvEntry = table.getEntry("tv");
    txEntry = table.getEntry("tx");
    tyEntry = table.getEntry("ty");
    tidEntry = table.getEntry("tid");

    Pose = new TagPoseCalc(tagYaw, tagPitch, x_offset, y_offset, id,Rotation2d.fromDegrees(gyro.getAngle()), RobotContainer.isRed);

    field = new Field2d();
    
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Fetch Limelight data
    if(tvEntry.getDouble(0) != 0) {
      tagYaw = txEntry.getDouble(0);
      tagPitch = tyEntry.getDouble(0);
      id = tidEntry.getDouble(0);
      Pose.updatePosValues(tagYaw, tagPitch, x_offset, y_offset, id,Rotation2d.fromDegrees(gyro.getAngle()), RobotContainer.isRed);
      Pose2d pose = getRobotPose();
      field.setRobotPose(pose);
      RobotContainer.chassis.updateVisionPose(pose);
    }
  }
  /**
   * returns the position of the robot
   * @return pose of robot
   */
  public Pose2d getRobotPose(){
    Pose2d robotPose = Pose.calculatePose();
    if (robotPose != null) {
      return robotPose;
    }
    return new Pose2d();
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
    builder.addDoubleProperty("dist from cam", ()->Pose.GetDistFromCamera(), null);
  }

}
