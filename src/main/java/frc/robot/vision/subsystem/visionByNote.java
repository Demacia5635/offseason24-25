// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;


import static frc.robot.vision.ConstantsVision.*;

import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.utils.NotePoseCalc;

public class VisionByNote extends SubsystemBase {
  /** Creates a new vision. */
   // NetworkTable for Limelight communication
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry tv;
  //private NetworkTableEntry tcorn;
  
  
  // Limelight data
  //private double x_offset;
  //private double y_offset;
  private double noteYaw;
  private boolean seeNote = false;
  //private boolean isClose = false;

  //private double[] corners;
  //private Pose2d robotPose;

//  private double[] corners = new double[8];
//  private double[] array = new double[8];

//  private double widthInAngle = 0;
//  private double widthInPix = 0;

  // Pose and distance calculation utilities
  // private NotePoseCalc notePose;

  //pose of robot in field 
  Field2d field;

  public VisionByNote(Pose2d robotPose) {
    //this.robotPose = robotPose;

    // Initialize Field2d for visualization

    //this.x_offset = NoteLimelightXOfset;
    //this.y_offset = NoteLimelightYOfset;    

    // Get the Limelight NetworkTable
    table = NetworkTableInstance.getDefault().getTable(NOTE_TABLE);
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    //tcorn = table.getEntry("tcornxy");
    //corners = new double[]{0,0,0,0,0,0,0,0};
  //  notePose = new NotePoseCalc(noteYaw, x_offset, y_offset, robotPose);
    
  //  field = new Field2d();

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Fetch Limelight data
    seeNote = tv.getDouble(0) != 0;
   
    noteYaw =  -tx.getDouble(0);

    //corners = tcorn.getDoubleArray(new double[]{0,0,0,0,0,0,0,0});
    //isClose = Math.abs(corners[0]-corners[2]) < 120;
    
    

    //widthInPix = Math.abs(corners[0]-corners[2]);
    //widthInAngle = Math.abs((corners[0]*ANGLE_PER_PIX_X)-(corners[2]*ANGLE_PER_PIX_X));
    //update Pose
    //notePose.update(noteYaw, x_offset, y_offset, robotPose);''

    //field.setRobotPose(getNotePose());;
  }
  /**
   * returns the position of the note
   * @return pose of note
   */
  // public Pose2d getNotePose(){
  //   Pose2d NotePose = notePose.calculatePose();
  //   if (NotePose != null) {
  //     return NotePose;
  //   }
  //   return new Pose2d();
  // }

  /**
   * 
   * @return the distens from robot to not(getNoorm);
   */
  // public double getDistToNote(){
  //   return notePose.getRobotToNote().getNorm();
  // }
  /**
   * 
   * @return the angle from robot to note(getAngle);
   */
  // public Rotation2d getAngleToNote(){
  //   return notePose.getRobotToNote().getAngle();
  // }

  public double getNoteYaw(){
    return noteYaw;
  }
  public boolean seeNote() {
    return seeNote;
  }
  // public boolean isNoteClose(){
  //   return isClose;
  // }


  @Override
  public void initSendable(SendableBuilder builder) {
      //SmartDashboard.putData("field-note", field);
      //builder.addDoubleProperty("pix", ()->Math.abs(corners[0]-corners[2]), null); 
  }

}
