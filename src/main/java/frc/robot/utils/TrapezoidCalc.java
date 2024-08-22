// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;


import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TrapezoidCalc {
    double lastVel = 0;
    double lastAcc = 0;
    double lastTime = 0;
    
    public TrapezoidCalc(){
    }

    /**
     * @param currentVelocity the current volocity of the object
     * @param maxVel will always be postive
     * @param endVel was only tested as 0
     * @param acc will always be postive
     * @param dis is the remaing distance to the end
     * 
     * @return the needed velocity
     */
    public double trapezoid(double currentVelocity, double maxVel, double endVel, double acc, double dis){
        
        double time = Timer.getFPGATimestamp();
        if (time - lastTime < 0.04) {
            if (lastAcc > 0 && currentVelocity < lastVel) {
                currentVelocity = lastVel;
            }
            if (lastAcc < 0 && currentVelocity > lastVel) {
                currentVelocity = lastVel;
            }
        }

        double timeToAccelerate = (currentVelocity-endVel)/acc;

        if (dis > 0) {
            double accelDistance = currentVelocity*timeToAccelerate + acc*Math.pow(timeToAccelerate,2)/2;
            if (dis > accelDistance) {
                lastVel = Math.min(currentVelocity + 0.02*acc, maxVel);
            } else {
                lastVel = currentVelocity - acc*0.02;
            }
        }

        else {
            double accelDistance = currentVelocity*timeToAccelerate - acc*Math.pow(timeToAccelerate,2)/2;
            if (dis < accelDistance) {
                lastVel =  Math.max(currentVelocity - 0.02*acc, -maxVel);
            } else {
                lastVel = currentVelocity + acc*0.02;
            }
        }

        lastTime = time;
        lastAcc = lastVel - currentVelocity;
        return lastVel;

    }
}