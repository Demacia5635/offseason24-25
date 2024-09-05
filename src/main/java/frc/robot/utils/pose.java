// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;


import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class pose{
  private String[] objects;
  private double[] dists;
  private double[] angles;
  private Translation2d pose;

  public pose(String[] objects, double[] dists, double[] angles) {
    this.objects = objects;
    this.dists = dists;
    this.angles = angles;

  }

  public Translation2d calcMyPose(){
    for (int i = 0; i < objects.length; i++) {
      Translation2d obj = Constants.dic.get(objects[i]);
      if (obj != null){
        if (angles[i]>=0 && angles[i]<=90){
          Translation2d trigo = trigo(dists[i], angles[i]);
          Translation2d point = new Translation2d(obj.getX() - trigo.getX(), obj.getY() - trigo.getY());
          pose = average(pose, point);
          
        }else if (angles[i]>=91 && angles[i]<=180){
          Translation2d trigo = trigo(dists[i], 180-angles[i]);
          Translation2d point = new Translation2d(obj.getX() + trigo.getX(), obj.getY() - trigo.getY());
          pose = average(pose, point);
          
        }else if (angles[i]>=181 && angles[i]<=270){
          Translation2d trigo = trigo(dists[i], 180+angles[i]);
          Translation2d point = new Translation2d(obj.getX() + trigo.getX(), obj.getY() + trigo.getY());
          pose = average(pose, point);

        }else if (angles[i]>=271 && angles[i]<=360){
          Translation2d trigo = trigo(dists[i], 360-angles[i]);
          Translation2d point = new Translation2d(obj.getX() - trigo.getX(), obj.getY() + trigo.getY());
          pose = average(pose, point);

        }
      }
    }
    return pose;
  }

  private Translation2d trigo(double dist, double angle){
    double m = dist*Math.tan(angle);
    double l = dist;

    return new Translation2d(l, m);
  }

  public Translation2d average(Translation2d p1, Translation2d p2){
    return ((p1 == null) ? p2 : ((p2 == null) ? p1 :(new Translation2d((p1.getX() + p2.getX())/2, (p1.getY() + p2.getY())/2))));
}
}
