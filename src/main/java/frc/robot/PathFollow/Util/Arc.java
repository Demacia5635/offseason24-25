// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.subsystems.chassis.ChassisConstants.*;

/** Add your docs here. */
public class Arc extends Segment{

    //p1 represents the start point, p2 represents the circle center
    Rotation2d angle;


    final Translation2d startVector;
    final double radius;
    /**
     * 
     * @param p1 - Start point of arc
     * @param p2 - Circle center of arc
     * @param angle - Arc's angle
     */
    public Arc(Translation2d p1, Translation2d p2, Rotation2d angle, boolean isAprilTagMode)
    {
        //start point
        super(p1,p2, isAprilTagMode);
        this.angle = angle;

        startVector = p1.minus(p2);
        radius = startVector.getNorm();
        
    }

    @Override
    public Translation2d[] getPoints()
    {
      Translation2d arrow = p1.minus(p2);
      Translation2d[] points = new Translation2d[4];
      double diffAngle = angle.getRadians();
      int place = 0;
      if(radius == 0){
        return new Translation2d[] {p1, p2};
      }
        for (double i = 0;Math.abs(i) < Math.abs(diffAngle); i = i + (diffAngle / 4)) {
            points[place] = p2.plus(arrow.rotateBy(new Rotation2d(i)));
          
            place++;
        }

        return points;
    }

    @Override
    public Translation2d calc(Translation2d pos,double velocity)
    {

        if(isAprilTagMode()) velocity = Math.min(velocity, 1);
        Translation2d relativePos = pos.minus(p2);
        double dFromCenter = relativePos.getNorm();

        Rotation2d tAngle = new Rotation2d(((velocity * CYCLE_DT) / radius) * Math.signum(angle.getDegrees()));


        //tangent angle to arc, determined by the robot's position
        Rotation2d tanAngle = relativePos.getAngle().plus(new Rotation2d(Math.toRadians(90 * Math.signum(angle.getDegrees()))));
        //fix angle = turn angle, multiplied by a ratio.
        //bigger ratio - will turn more towards the center
        //smaller ratio - will turn less towards the center
        Rotation2d fixAngle = tAngle.times(dFromCenter / radius);
      return new Translation2d(velocity, tanAngle.plus(fixAngle));
    }

    @Override
    public double distancePassed(Translation2d pos)
    {
      Translation2d relativePos = pos.minus(p2);

      Rotation2d diffAngle = startVector.getAngle().minus(relativePos.getAngle());

      return Math.abs(diffAngle.getRadians() * radius);
    }

   // public 


    @Override
    public double getLength()
    {
      return Math.abs(angle.getRadians()) * radius;
    }

    @Override
    public String toString() {
        return "\n~Arc~\nStartPoint : " + p1 + "\nCircleCenter : " + p2 + "\nAngle : " + angle + "\nRadius : " + radius;
    }

}
