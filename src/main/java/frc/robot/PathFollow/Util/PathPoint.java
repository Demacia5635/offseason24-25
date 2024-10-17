// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * 
 *A point in the path
 *
 */
public class PathPoint extends Pose2d{
    double radius;
    boolean isAprilTag;

    public PathPoint(Translation2d p, Rotation2d r) {
      this(p.getX(),p.getY(),r,0);
    }
    
    public PathPoint(Translation2d p, Rotation2d r, double radius) {
      this(p.getX(),p.getY(),r,radius);
    }
    public PathPoint(double x, double y, Rotation2d rotation, double radius) {
      super(x,y,rotation);
      this.radius = radius;
    }

    public double getRadius()
    {
      return radius;
    }

    public void setRadius(double radius)
    {
      this.radius = radius;
    }
    public boolean isAprilTag(){
      return isAprilTag;
    }


}
