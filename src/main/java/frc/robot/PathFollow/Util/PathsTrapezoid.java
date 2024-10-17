// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

/** Add your docs here. */
public class PathsTrapezoid {
    private final double t = 0.02;

    private double maxVel;
    private double accel;
    private double deltaV;
    private double deAccelParam;

    public PathsTrapezoid(double maxVel, double accel){
        this.accel = accel;
        this.maxVel = maxVel;
        this.deltaV = accel * t;
        this.deAccelParam = 2;
    }

    

    private double getDistanceAccel(double curVel){
        return (curVel * t) + (0.5 * (accel / deAccelParam) * t * t);
    }
    private double getDistanceCruise(double curVel){
        return curVel * t;
    }

    private boolean isAbleToAccel(double wantedVel, double distanceLeft){
        return getDistanceAccel(wantedVel) < distanceLeft;
    }
    private boolean isAbleToCruise(double curVel, double distanceLeft){
        return getDistanceCruise(curVel) < distanceLeft;
    }
    private boolean isMaxVel(double wantedVel){
        return Math.abs(wantedVel) <= Math.abs(maxVel - (t/2 * maxVel));
    }
    public double calc(double distanceLeft, double currentVel, double targetVel){
        if(distanceLeft < 0) {
            return -calc(-distanceLeft, -currentVel, -targetVel);
        }
        if(isMaxVel(currentVel + deltaV) && isAbleToCruise(maxVel, distanceLeft)) return maxVel;
        else if(isAbleToAccel(currentVel + deltaV, distanceLeft)) return currentVel + deltaV;
        else {
            return Math.max(currentVel - (deltaV * deAccelParam), targetVel);
        }
    }
    

}
