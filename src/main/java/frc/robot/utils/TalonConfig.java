package frc.robot.utils;

/** 
 * Class to hold all Talon FX/SRX configuration
 * Applicable to Phoenix 6
 *  */
public class TalonConfig {
    public int id;                  // Canbus ID
    public String canbus;           // Canbus name
    public String name;             // Name of the motor - used to logging
    public double maxVolt = 12;     // Max Volt allowed
    public double minVolt = -12;    // Min Vols allowed
    public double maxCurrent = 40;  // Max current allowed
    public double maxCurrentTreshold = 42; // Current limit will be applied once the current reach this level
    public double maxCurrentTriggerTime = 0.2; // Current limit will be applied once the current reach the limit current for at least this time
    public double rampUpTime = 0.3;   // max power change time from 0 to full. 
    public boolean brake = true;    // brake/coast
    public double motorRatio = 1;   // motor to mechanism ratio
    public boolean inverted = false; // if to invert motor
    public closeLoopParam pid; // close loop argument - PID + FF
    public closeLoopParam pid1 = null; // pid for slot 1
    public closeLoopParam pid2 = null; // pid for slot 2
    public double motionMagicAccel = 10; // maximum motion magic (position) acceleration
    public double motionMagicVelocity = 1; // maximum motition magic velocity
    public double motionMagicJerk = 10;    // maximum motion magic jerk
    public double kv2 = 0;
    public double ksin = 0;
    public double posToRad = 0;

    /** 
    * Class to hold closed loop param
    *  */
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

    /** 
     * Constructor
     * @param id - canbus ID
     * @param canbus - Name of canbus
     * @param name - name of motor for logging
     */
    public TalonConfig(int id, String canbus, String name) {
        this.id = id;
        this.canbus = canbus;
        this.name = name;
    }

    
    /** 
     * @param maxVolt
     * @param minVolt
     * @return TalonConfig
     */
    public TalonConfig withVolts(double maxVolt, double minVolt) {
        this.maxVolt = maxVolt;
        this.minVolt = minVolt;
        return this;
    }
    
    /** 
     * @param maxCurrent
     * @param treshold
     * @param trigerTime
     * @return TalonConfig
     */
    public TalonConfig withCurrent(double maxCurrent, double treshold, double trigerTime) {
        this.maxCurrent = maxCurrent;
        this.maxCurrentTreshold = treshold;
        this.maxCurrentTriggerTime = trigerTime;
        return this;
    }

    
    /** 
     * @param brake
     * @return TalonConfig
     */
    public TalonConfig withBrake(boolean brake) {
        this.brake = brake;
        return this;
    }

    /** 
     * @param invert
     * @return TalonConfig
     */
    public TalonConfig withInvert(boolean invert) {
        this.inverted = invert;
        return this;
    }

    /** 
     * @param rampTime
     * @return TalonConfig
     */
    public TalonConfig withRampTime(double rampTime) {
        this.rampUpTime = rampTime;
        return this;
    }

    /** 
     * @param ratio - motor to mechanism ratio
     * @return TalonConfig
     */
    public TalonConfig withMotorRatio(double ratio) {
        this.motorRatio = ratio;
        return this;
    }

    /** 
     * @param kv2
     * @param ksin
     * @param posToRad
     * @return TalonConfig
     */
    public TalonConfig withFeedForward(double kv2, double ksin, double posToRad) {
        this.kv2 = kv2;
        this.ksin = ksin;
        this.posToRad = posToRad;
        return this;
    }

    /** 
     * @param kp
     * @param ki
     * @param kd
     * @param ks
     * @param kv
     * @return TalonConfig
     */
    public TalonConfig withPID(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        pid = new closeLoopParam(kp, ki, kd, ks, kv, ka, kg);
        return this;
    }



    /** 
     * @param kp
     * @param ki
     * @param kd
     * @param ks
     * @param kv
     * @return TalonConfig
     */
    public TalonConfig withPID1(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        pid1 = new closeLoopParam(kp, ki, kd, ks, kv, ka, kg);
        return this;
    }

    /** 
     * @param kp
     * @param ki
     * @param kd
     * @param ks
     * @param kv
     * @return TalonConfig
     */
    public TalonConfig withPID2(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        pid2 = new closeLoopParam(kp, ki, kd, ks, kv, ka, kg);
        return this;
    }    
    
    /** 
     * @param velocity
     * @param acceleration
     * @param jerk
     * @return TalonConfig
     */
    public TalonConfig withMotionMagic(double velocity, double acceleration, double jerk) {
        motionMagicVelocity = velocity;
        motionMagicAccel = acceleration;
        motionMagicJerk = jerk;
        return this;
    }


}