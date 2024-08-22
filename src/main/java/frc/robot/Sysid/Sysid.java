package frc.robot.Sysid;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;



    /** 
     * Class to calculate feed forward gains for velocity/acceleration control
     * there are different gains - the default are 
     *      KS (times signum(v))
     *      KV (times v)
     *      KA (times v-lastV)
     * Others:
     *      K1      (times 1)
     *      KRad    (times angle of mechanism in Radians)
     *      KMeter  (times position of mechanism in Meters)
     *      KV2     (times v squared)
     *      KVsqrt  (times sqrt of v)
     *      KSin    (times sin(radians))
     *      KCos    (times cos(radians))
     *      KTan    (times tan(radians))
     * 
     * It operates the mechanism/motors at different power setting for a duration 
     * It collect the velocity and optionaly radians and position of the mechanism
     * It than calculate the best fit gains 
     * 
     */
public class Sysid {


    /**
     * Gains enum - type of gains
     */
    public static enum Gains { K1, KS, KV, KA, KRad, KMeter, KV2, KVsqrt, KSin, KCos, KTan; }

    Consumer<Double> setPower;      // function to set the power
    DataCollector dataCollector;    // the data collector class
    double minPower;
    double maxPower;
    double deltaPower;              // the change of power between power cycles 
    int nPowerCycles;               // how many powers to use (the system will run each power in positive and negative values)
    double powerCycleDuration;      // how long each power cycle
    double powerCycleDelay;         // delay between power cycles
    Subsystem[] subsystems;         // for add requirements
    static double defaultDuration = 2.5;
    static double defaultDelay = 10;
    Gains[] gains;                  // the gains we are looking for
    double[] result=null;           // the result, after analyze
    boolean steadyOnly = false;     // if we need steady only

    /**
     * Constructor with default values - anly required the setPower, setVelocity, min/max power and subsystems
     * @param setPower
     * @param getVelocity
     * @param minPower
     * @param maxPower
     * @param subsystems
     */
    public Sysid(Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            double minPower,
            double maxPower,
            Subsystem... subsystems) {
        this(new Gains[] { Gains.KS, Gains.KV, Gains.KA, Gains.KV2},
                setPower,
                getVelocity,
                null,
                null,
                minPower,
                maxPower,
                3,
                defaultDuration,
                defaultDelay,
                subsystems);
    }

    /**
     * Constructor with additional values - duration and delay
     * @param setPower
     * @param getVelocity
     * @param minPower
     * @param maxPower
     * @param subsystems
     */
    public Sysid(Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            double minPower,
            double maxPower,
            double powerCycleDuration,
            double powerCycleDelay,
            Subsystem... subsystems) {
        this(new Gains[] { Gains.KS, Gains.KV, Gains.KA },
                setPower,
                getVelocity,
                null,
                null,
                minPower,
                maxPower,
                3,
                powerCycleDuration,
                powerCycleDelay,
                subsystems);
    }

    /**
     * Constructor with all parameters
     * @param setPower
     * @param getVelocity
     * @param minPower
     * @param maxPower
     * @param subsystems
     */
    public Sysid(Gains[] types,
            Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            Supplier<Double> getRadians,
            Supplier<Double> getMeter,
            double minPower,
            double maxPower,
            int nPowerCycles,
            double powerCycleDuration,
            double powerCycleDelay,
            Subsystem... subsystems) {

        this.setPower = setPower;
        dataCollector = new DataCollector(types, getVelocity, getRadians, getMeter, nPowerCycles, powerCycleDuration);
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.nPowerCycles = nPowerCycles;
        this.powerCycleDelay = powerCycleDelay;
        this.powerCycleDuration = powerCycleDuration;
        deltaPower = (maxPower - minPower) / (nPowerCycles - 1);
        this.subsystems = subsystems;
        this.gains = types;
    }

    public static Command getSteadyCommand(Consumer<Double> setPower, Supplier<Double> getVelocity, double minPower, double maxPower, double powerStep, double minVelocity, double maxVelocity, Subsystem...subsystems) {
        Sysid id = new Sysid(new Gains[] {Gains.KS, Gains.KV, Gains.KV2}, setPower, getVelocity, null, null, minPower, maxPower, 2, 1, 1,subsystems);
        id.steadyOnly = true;
        Command cmd  = new NoAccelerationPowerCommand(setPower, minPower, maxPower, powerStep, id.dataCollector, false, minVelocity, maxVelocity, maxVelocity, subsystems);
        return cmd.andThen(new InstantCommand(() -> id.analyze()));
    }


    /**
     * run the command
     */
    public void run() {
        getCommand().schedule();
    }

    
    /** 
     * calculate the power for cycle
     * cycles run - minPower, -minPower, (minPower+delta), -(minPower+delts).....(maxPower), -max(Power)
     * @param cycle
     * @return double
     */
    double power(int cycle) {
        int p = cycle / 2;
        double sign = cycle % 2 == 0 ? 1 : -1;
        return sign * (minPower + p * deltaPower);
    }

    
    /** 
     * Generate the command to run the system at different powers, collect the data and analyze the result
     * there is a delay between each power cycle
     * @return Command
     */
    public Command getCommand() {
        boolean resetDataCollector = true;
        Command cmd = new WaitCommand(powerCycleDelay);
        for (int c = 0; c < nPowerCycles; c++) {
            double power = minPower + c * deltaPower;
            cmd = cmd.andThen(getPowerCommand(power, resetDataCollector));
            resetDataCollector = false;
            cmd = cmd.andThen(getPowerCommand(-power, resetDataCollector));
        }
        return cmd.andThen(new InstantCommand(() -> analyze()));
    }

    public Command getCommandOneWay() {
        boolean resetDataCollector = true;
        Command cmd = new WaitCommand(powerCycleDelay);
        for (int c = 0; c < nPowerCycles; c++) {
            double power = minPower + c * deltaPower;
            cmd = cmd.andThen(getPowerCommand(power, resetDataCollector));
            resetDataCollector = false;
//            cmd = cmd.andThen(getPowerCommand(-power, resetDataCollector));
        }
        return cmd.andThen(new InstantCommand(() -> analyze()));
    }

    /**
     * Get the command for a power - with the duration and delay
     */
    Command getPowerCommand(double power, boolean resetDataCollector) {
        return ((new PowerCycleCommand(setPower, power, dataCollector, resetDataCollector, subsystems))
                    .withTimeout(powerCycleDuration)).andThen(new WaitCommand(powerCycleDelay));
    }

    /**
     * Analyze the result
     * Using data collector solve
     * It also calculate the worst error and the avg error squared
     */
    void analyze() {
        SimpleMatrix coef = dataCollector.solve();
        result = new double[gains.length];
        for (int i = 0; i < gains.length; i++) {
            result[i] = coef.get(i, 0);
            SmartDashboard.putNumber("SysID/" + gains[i], result[i]);
           // System.out.println("Sysid: " + gains[i] + " = " + result[i]);
        }
        SimpleMatrix p = dataCollector.data().mult(coef);
        SimpleMatrix e = dataCollector.power().minus(p);
        SimpleMatrix ee = e.elementMult(e);
        double max = Math.sqrt(ee.elementMax());
        double avg = ee.elementSum() / ee.getNumRows();
        SmartDashboard.putNumber("Sysid/Max Error", max);
        SmartDashboard.putNumber("Sysid/Avg Error Sqr", avg);
       // System.out.println("Sysid: max error=" + max + " avg error squared=" + avg);
        double kp = (valueOf(Gains.KV, gains, result) + valueOf(Gains.KA, gains, result))/5.0;
        SmartDashboard.putNumber("Sysid/KP (Roborio)", kp);
        SmartDashboard.putNumber("Sysid/KP (Roborio)", kp);
    }

    /**
     * Value of a specific Gain type
     * @param gain
     * @param gains
     * @param values
     * @return
     */
    double valueOf(Gains gain, Gains[] gains, double[] values) {
        for(int i = 0; i < gains.length; i++) {
            if(gains[i] ==gain) {
                return values[i];
            }
        }
        return 0;
    }

    /**
     * 
     * @return result array of gain values
     */
    public double[] result() {
        return result;
    }

    /**
     * 
     * @return result array of gain types
     */
    public Gains[] gains() {
        return gains;
    }
    
}