// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.vision.utils.NotePoseCalc;

public class visionByNote extends SubsystemBase {
  /** Creates a new vision. */
   // NetworkTable for Limelight communication
  private NetworkTable table;
  
  // Limelight data
  private double x_offset;
  private double y_offset;
  private double noteYaw;
  private double notePitch;
  private Pose2d robotPose;

  // Pose and distance calculation utilities
  private NotePoseCalc notePose;

  //pose of robot in field 
  Field2d field;

  public visionByNote(Pose2d robotPose) {
    this.robotPose = robotPose;

    // Initialize Field2d for visualization

    this.x_offset = Constants.NoteLimelightXOfset;
    this.y_offset = Constants.NoteLimelightYOfset;    

    // Get the Limelight NetworkTable
    table = NetworkTableInstance.getDefault().getTable(Constants.NoteTable);
    notePose = new NotePoseCalc(noteYaw, notePitch, x_offset, y_offset, robotPose, false);
    
    field = new Field2d();

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Fetch Limelight data
    noteYaw = table.getEntry("tx").getDouble(0);
    notePitch = table.getEntry("ty").getDouble(0);
    noteYaw *=-1;
    //update Pose
    notePose.update(noteYaw, notePitch, x_offset, y_offset, robotPose, false);

    field.setRobotPose(getNotePose());;
  }
  /**
   * returns the position of the note
   * @return pose of note
   */
  public Pose2d getNotePose(){
    Pose2d NotePose = notePose.calculatePose();
    if (NotePose != null) {
      return NotePose;
    }
    return new Pose2d();
  }

  /**
   * 
   * @return the distens from robot to not(getNoorm);
   */
  public double getDistToTag(){
    return notePose.getRobotToNote().getNorm();
  }
  /**
   * 
   * @return the angle from robot to note(getAngle);
   */
  public Rotation2d getAngleTag(){
    return notePose.getRobotToNote().getAngle();
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      SmartDashboard.putData("field-note", field);
  }

}
