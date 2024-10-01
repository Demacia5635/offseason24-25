// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class TalonConfig {
    public int id;                  // Canbus ID
    public String canbus;           // Canbus name
    public double maxVolt = 12;     // Max Volt allowed
    public double minVolt = -12;    // Min Vols allowed
    public double maxCurrent = 40;  // Max current allowed
    public double maxCurrentTreshold = 42; // Current limit will be applied once the current reach this level
    public double maxCurrentTriggerTime = 0.2; // Current limit will be applied once the current reach the limit current for at least this time
    public double rampUpTime = 0.3;
    public boolean brake = true;    // brake/coast
    public double motorRatio = 1;   // motor to mechanism ratio
    public boolean inverted = false; // if to invert motor
    public closeLoopParam pid; // close loop argument - PID + FF
    public double motionMagicAccel = 10; // maximum motion magic (position) acceleration
    public double motionMagicVelocity = 1; // maximum motition magic velocity
    public double motionMagicJerk = 10;    // maximum motion magic jerk
    

    class closeLoopParam { // calculate volts - not -1 to 1 !!!
        double kp;  
        double ki;
        double kd;
        double ks;
        double kv;
        double ka;
        double kg;

        closeLoopParam(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
            this.ka = ka;
            this.kd = kd;
            this.ki = ki;
            this.kp = kp;
            this.ks = ks;
            this.kv = kv;
            this.kg = kg;
        }
    }

    public TalonConfig(int id, String canbus, String name) {
        this.id = id;
        this.canbus = canbus;
    }

    public TalonConfig withVolts(double maxVolt, double minVolt) {
        this.maxVolt = maxVolt;
        this.minVolt = minVolt;
        return this;
    }

    public TalonConfig withCurrent(double maxCurrent, double treshold, double trigerTime) {
        this.maxCurrent = maxCurrent;
        this.maxCurrentTreshold = treshold;
        this.maxCurrentTriggerTime = trigerTime;
        return this;
    }

    public TalonConfig withBrake(boolean brake) {
        this.brake = brake;
        return this;
    }

    public TalonConfig withInvert(boolean invert) {
        this.inverted = invert;
        return this;
    }

    public TalonConfig withRampTime(double rampTime) {
        this.rampUpTime = rampTime;
        return this;
    }

    public TalonConfig withMotorRatio(double ratio) {
        this.motorRatio = ratio;
        return this;
    }
}
