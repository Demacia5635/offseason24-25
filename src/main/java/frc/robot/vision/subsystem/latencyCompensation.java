// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class latencyCompensation extends SubsystemBase {
  
  private NetworkTable table;
  private Pose2d robotPose;

  //Latency (ms)
  private double pipelineLatency; // Pipeline's latency contribution
  private double imageCaptureLatency; // Capture pipeline latency (default 11 ms)
  private double totalLatency; // Total latency
  
  public latencyCompensation(Pose2d robotPose) {
    this.robotPose = robotPose;
    this.table = NetworkTableInstance.getDefault().getTable("limelight-tag");
  }

  @Override
  public void periodic() {
    // Fetch latency
    pipelineLatency = table.getEntry("tl").getDouble(0);
    imageCaptureLatency = table.getEntry("cl").getDouble(11);
    totalLatency = pipelineLatency + imageCaptureLatency;
  }

  public double getTotalLatency() {
    // Convert to seconds
    return totalLatency / 1000;
  }

  public Pose2d predictPosition(double velocity) {
    Rotation2d rotation = robotPose.getRotation();
    
    double predictedDistance = velocity * getTotalLatency(); // Distance traveled during latency

    double predictedX = robotPose.getX() + predictedDistance * Math.cos(rotation.getRadians());
    double predictedY = robotPose.getY() + predictedDistance * Math.sin(rotation.getRadians());

    return new Pose2d(predictedX, predictedY, rotation);
  }

  public double predictRotation(double currentAngle, double angularVelocity) {
    return currentAngle + (angularVelocity * getTotalLatency());
  }
}
