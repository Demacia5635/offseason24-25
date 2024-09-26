// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import static frc.robot.chassis.ChassisConstants.CYCLE_DT;

/** Add your docs here. */
public class TrapezoidNoam {
    double maxVelocity; // Maximum permissible velocity
    double maxAcceleration; // Maximum permissible acceleration
    private double deltaVelocity; // Velocity increment at each time step
    private double lastTime  = 0;
    private double lastV;
    private double lastA;
    public boolean debug = false;


    // Constructor to initialize with maximum velocity and acceleration
    public TrapezoidNoam(double maxVelocity, double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        // Velocity increment calculated based on max acceleration
        deltaVelocity = maxAcceleration * 0.02;
    }

    // Helper function to calculate distance required to change from current velocity to target velocity
    private double distanceToVelocity(double currentVelocity, double targetVelocity, double acceleration) {
        double t = (currentVelocity-targetVelocity) / maxAcceleration;
        if(t < 0) {
            return -currentVelocity*t + 0.5 * (maxAcceleration * t * t);
        } else {
            return currentVelocity*t - 0.5 * (maxAcceleration * t * t);
        }

    }

    // Function to calculate the next velocity setpoint, based on remaining distance and current and target velocities
    public double calculate(double remainingDistance, double curentVelocity, double targetVelocity) {
        double baseCurV = curentVelocity;
        // Case for negative remaining distance
        if(remainingDistance < 0) {
            return  -calculate(-remainingDistance, -curentVelocity, -targetVelocity);
        }

        double time = Timer.getFPGATimestamp();
        if(time - lastTime <= CYCLE_DT) {
            if(lastA > 0 && curentVelocity < lastV) {
                curentVelocity = lastV;
            } else if(lastA < 0 && curentVelocity > lastV) {
                curentVelocity = lastV;
            }
        }        // Case for below max velocity, and enough distance to reach targetVelocity at max acceleration
        if(curentVelocity < maxVelocity && 
            distanceToVelocity(curentVelocity+deltaVelocity, targetVelocity, maxAcceleration) < remainingDistance - cycleDistanceWithAccel(curentVelocity)) {
            
            lastV = Math.min(curentVelocity + deltaVelocity, maxVelocity);
        } 
        // Case for enough distance to reach targetVelocity without acceleration
        else if(distanceToVelocity(curentVelocity, targetVelocity, maxAcceleration) < remainingDistance - cycleDistanceNoAccel(curentVelocity)) {
            lastV = Math.min(curentVelocity, maxVelocity);
        } 
        // Case for not enough distance to reach targetVelocity, must decelerate
        else {
            double t = remainingDistance * 2 / (curentVelocity + targetVelocity);
            double a = (curentVelocity - targetVelocity)/t;
            lastV = Math.min(maxVelocity,curentVelocity - a*0.02);
          //  if(debug) System.out.println(" reduce v - " + a + " maxV=" + maxVelocity + " maxA=" + maxAcceleration);
        }
        if(debug) {
            // System.out.println(" Trap: curV = " + baseCurV + " / " + curentVelocity + " next=" + lastV + " remain=" + remainingDistance);
        }
        lastA = lastV - curentVelocity;
        lastTime = time;
        return lastV;
    }

    // Helper function to compute the distance travelled in one cycle without acceleration
    private double cycleDistanceNoAccel(double currentVelocity) {
        return currentVelocity * 0.02;
    }
    // Helper function to compute the distance travelled in one cycle with maximum acceleration
    private double cycleDistanceWithAccel(double currentVelocity) {
        return currentVelocity * 0.02 + (0.5*maxAcceleration * Math.pow(0.02, 2));
    }
}
