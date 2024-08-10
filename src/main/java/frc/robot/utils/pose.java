// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.Constants;

public class pose{
  private String[] objects;
  private double[] dists;
  private double[] angles;
  private Point pose;

  public pose(String[] objects, double[] dists, double[] angles) {
    this.objects = objects;
    this.dists = dists;
    this.angles = angles;

  }

  public Point calcMyPose(){
    for (int i = 0; i < objects.length; i++) {
      Point obj = Constants.dic.get(objects[i]);
      if (obj != null){
        if (angles[i]>=0 && angles[i]<=90){
          Point trigo = trigo(dists[i], angles[i]);
          Point point = new Point(obj.getX() - trigo.getX(), obj.getY() - trigo.getY());
          pose = average(pose, point);
          
        }else if (angles[i]>=91 && angles[i]<=180){
          Point trigo = trigo(dists[i], 180-angles[i]);
          Point point = new Point(obj.getX() + trigo.getX(), obj.getY() - trigo.getY());
          pose = average(pose, point);
          
        }else if (angles[i]>=181 && angles[i]<=270){
          Point trigo = trigo(dists[i], 180+angles[i]);
          Point point = new Point(obj.getX() + trigo.getX(), obj.getY() + trigo.getY());
          pose = average(pose, point);

        }else if (angles[i]>=271 && angles[i]<=360){
          Point trigo = trigo(dists[i], 360-angles[i]);
          Point point = new Point(obj.getX() - trigo.getX(), obj.getY() + trigo.getY());
          pose = average(pose, point);

        }
      }
    }
    return pose;
  }

  private Point trigo(double dist, double angle){
    double m = dist*Math.sin(angle);
    double l = dist*Math.cos(angle);

    return new Point(l, m);
  }

  public Point average(Point p1, Point p2){
    return ((p1 == null) ? p2 : ((p2 == null) ? p1 :(new Point((p1.getX() + p2.getX())/2, (p1.getY() + p2.getY())/2))));
}
}
