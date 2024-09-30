// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;


import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class Trapezoid {
    private static final double CYCLE_DT=0.02;
    double maxVelocity; // Maximum permissible velocity
    double maxAcceleration; // Maximum permissible acceleration
    private double deltaVelocity; // Velocity increment at each time step
    private double lastTime  = 0;
    private double deacceleratingOffset;
    private double lastV;
    private double lastA;


    // Constructor to initialize with maximum velocity and acceleration
    public Trapezoid(double maxVelocity, double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        // Velocity increment calculated based on max acceleration
        deltaVelocity = maxAcceleration * CYCLE_DT;
        // deaccelerartion offset - start deacceleration before 
        deacceleratingOffset = maxAcceleration*CYCLE_DT*CYCLE_DT;
    }

    // Helper function to calculate distance required to change from current velocity to target velocity
    private double  distanceToVelocity(double currentVelocity, double targetVelocity, double acceleration) {
        double deltaVelocity = currentVelocity - targetVelocity;
        double time = Math.abs(deltaVelocity/acceleration);
        return (currentVelocity + targetVelocity)/2*time; // avg velocity * time
    }

    public double calculate(double remainingDistance, double curentVelocity, double targetVelocity) {
        return calculate(remainingDistance, curentVelocity, targetVelocity, false);
    }
    // Function to calculate the next velocity setpoint, based on remaining distance and current and target velocities
    public double calculate(double remainingDistance, double curentVelocity, double targetVelocity, boolean debug) {
        // Case for negative remaining distance
        if(remainingDistance < 0) {
            return  -calculate(-remainingDistance, -curentVelocity, -targetVelocity, debug);
        }
        double time = Timer.getFPGATimestamp();
        if(time-lastTime < CYCLE_DT) {
            return lastV;
        }
        double cv = curentVelocity;
        // update cv for situation that we are not accelerating or deaccelerating as expected
//        if(time - lastTime < Constants.CYCLE_DT*2) {
//            if(lastA > 0 && curentVelocity < lastV) { // 
//                cv = lastV;
//            } else if(lastA < 0 && curentVelocity > lastV) {
//                cv = lastV;
//            }
//        }        
        // Case for below max velocity, and enough distance to reach targetVelocity at max acceleration
        if(cv < maxVelocity && distanceToVelocity(cv+deltaVelocity, targetVelocity, maxAcceleration) < remainingDistance - cycleDistanceWithAccel(cv)) {
            lastV = Math.min(curentVelocity + deltaVelocity, maxVelocity);
         

        } 
        // Case at max velocity and not deacceleration yet
        else if(cv >= maxVelocity && distanceToVelocity(maxVelocity, targetVelocity, maxAcceleration) < remainingDistance - cycleDistanceNoAccel(cv)) {
            lastV = maxVelocity;
          } 
        // case we need to accelerate at a slower rate
        else {
            double distanceToDeaccelerarion = remainingDistance - distanceToVelocity(curentVelocity, targetVelocity, maxAcceleration); 
          
            if(distanceToDeaccelerarion + deacceleratingOffset > 0  || curentVelocity < targetVelocity) {
                // need to accelerate at a lower value
                // distanceToDeaccelerarion -= cycleDistanceNoAccel(curentVelocity);
                lastV = curentVelocity + distanceToDeaccelerarion/CYCLE_DT/2;
            

            } else { // deaccelration
                // calclate the required deacceleration from cv to target v in remaining distance
                double avgVelocity = (cv + targetVelocity) / 2;
                double deaccelerartionTime = remainingDistance / avgVelocity;
                double deaccelerartion = (cv-targetVelocity)/deaccelerartionTime;
                lastV = cv - deaccelerartion*CYCLE_DT;
            
            }
        }
        lastA = lastV - curentVelocity;
        lastTime = time;
        return lastV;
    }

    // Helper function to compute the distance travelled in one cycle without acceleration
    private double cycleDistanceNoAccel(double currentVelocity) {
        return currentVelocity * CYCLE_DT;
    }
    // Helper function to compute the distance travelled in one cycle with maximum acceleration
    private double cycleDistanceWithAccel(double currentVelocity) {
        return currentVelocity * CYCLE_DT + (0.5*maxAcceleration * Math.pow(CYCLE_DT, 2));
    }
}