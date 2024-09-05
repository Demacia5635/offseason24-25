// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Lengtandangle;
import frc.robot.utils.pose;

public class subpose extends SubsystemBase {
  /** Creates a new subpose. */
  private NetworkTable table;
  private double tx;
  private double ty;
  private double id;
  private pose pose;
  private Lengtandangle dist;
  private Field2d field;

  private String[] objects;
  private double[] dists;
  private double[] angles;

  public subpose() {

    

    SmartDashboard.putData("RobotContainer", this);

  }

    public void initSendable(SendableBuilder builder){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    id = table.getEntry("id").getDouble(0);

    dist = new Lengtandangle((int)id, tx, ty);
    for (int i = 0; i < 0; i++) {
      objects[i] = dist.GetObj();
      dists[i] = dist.GetDist();
      angles[i] = dist.GetAngle();
    }
    pose = new pose(objects, dists, angles);
    field = new Field2d();
    field.setRobotPose(new Pose2d(pose.calcMyPose(),new Rotation2d()));

    SmartDashboard.putData("field", field);

  }
}
