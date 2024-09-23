package frc.robot.Sysid;

import java.util.function.Supplier;
import org.ejml.simple.SimpleMatrix;
import static frc.robot.Sysid.Sysid.Gains;

/**
 * Data collector class
 * 
 * Store the data in simple matrix (n X number of gains)
 * Store the power in a different simple matrx (n X 1)
 * Uses supplied get function to get the velocity and other optional data (radinas and position)
 * calculate the best gains
 */
public class DataCollector {

    SimpleMatrix data;
    SimpleMatrix powerRange20;
    SimpleMatrix powerRange50;
    SimpleMatrix powerRange70;
    int nextRow;
    Gains[] gains;
    Supplier<Double> getVelocity;
    Supplier<Double> getRadians;
    Supplier<Double> getMeter;
    int nPowerCycles;
    double powerCycleDuration;
    double lastV = 0;

    /**
     * Constructor with all required data
     * @param gains
     * @param getVelocity
     * @param getRadians
     * @param getMeter
     * @param nPowerCycles
     * @param powerCycleDuration
     */

    public DataCollector(Gains[] gains, Supplier<Double> getVelocity, Supplier<Double> getRadians, Supplier<Double> getMeter, int nPowerCycles, double powerCycleDuration) {
        this.gains = gains;
        this.getVelocity = getVelocity;
        this.getRadians = getRadians;
        this.getMeter = getMeter;
        this.nPowerCycles = nPowerCycles;
        this.powerCycleDuration = powerCycleDuration;
        int matrixRows = (int)(nPowerCycles*2*powerCycleDuration/0.02) + 100; // the maximum number of rows we will need + safety value
        data = new SimpleMatrix(matrixRows, gains.length);
        powerRange20 = new SimpleMatrix(matrixRows, 1);
        nextRow = 0;
        lastV = 0;
    }

    /**
     * Function to collect the data
     * Adding the values for each required gain type
     * Adding the power to the right power matrix which is between the ranges of 1-100 (including minus)
     * @param power
     */
    public void collect(double power) {
        if (power>=0) {
            double powerInPerecent = Math.abs(power*100);
            if(powerInPerecent <= 100 && powerInPerecent >= 70){
                this.powerRange70.set(nextRow, 0, power);
            }

            else if(powerInPerecent <= 69 && powerInPerecent >= 20){
                this.powerRange50.set(nextRow, 0, power);
            }

            else{
                this.powerRange20.set(nextRow, 0, power);
            }

            double v = getVelocity.get();
            double rad = getRadians != null? getRadians.get():0;
            double meter = getMeter != null? getMeter.get():0;
            for(int i = 0; i < gains.length; i++) {
                data.set(nextRow, i, value(gains[i], v, rad, meter));
            }
            lastV = v;
            nextRow++;   
        }
        
    }
    

    /**
     * Calculate the applicable value based on the gain type and provided data
     * @param gain
     * @param v
     * @param rad
     * @param meter
     * @return applicable value
     */
    double value(Gains gain, double v, double rad, double meter) {
        switch (gain) {
            case K1:
                return 1;
            case KS:
                return Math.signum(v);
            case KV:
                return v;
            case KA:
                return v - lastV;
            case KRad:
                return rad;
            case KMeter:
                return meter;
            case KCos:
                return Math.cos(rad);
            case KSin:
                return Math.sin(rad);
            case KTan:
                return Math.tan(rad);
            case KV2:
                return v*v;
            case KVsqrt:
                return Math.sqrt(Math.abs(v));
            default:
                return 0;
        }
    }

    /**
     * Function to set the lastV
     */
    public void resetLastV() {
        lastV = getVelocity.get();
    }

    /**
     * Function to reset the data 
     */
    public void resetData() {
        nextRow = 0;
    }

    /**
     * 
     * @return the colllected data
     */
    public SimpleMatrix data() {
        return data.rows(0, nextRow);
    }

    /**
     * 
     * @return the collected power
     */
    public SimpleMatrix power() {
        return powerRange20.rows(0, nextRow);
    }

    /**
     * 
     * @return the gains matrix
     */
    public SimpleMatrix solve() {
        return data().solve(power());
    }

    /**
     * 
     * @return the array of gains type
     */
    public Gains[] gains() {
        return gains;
    }

}