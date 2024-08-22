package frc.robot.Sysid;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class to run a sysid power cycle
 * It just set the power once - in initialize
 * It collect the data each cycle
 * 
 */
public class NoAccelerationPowerCommand extends Command {

    double power;                   // the power to use
    DataCollector dataCollector;    // data collector
    Consumer<Double> setPower;      // set power function
    boolean init;                   // if init
    boolean resetDataCollector;     // reset the data collector data for rerun of the same command group
    double minVelocity;             // min velocity to collect
    double maxVelocity;             // max velocity - end command when reached
    double maxCycleVelocityChange;  // steady state is when 5 cycles we have velocity change smaller 
    double minPower;                // min power to use
    double maxPower;                // max power to use
    double powerStep;               // power step
    int steadyCount;                // count cycle with small velocity change
    double lastV;                   // the last velocity

    public NoAccelerationPowerCommand(Consumer<Double> setPower, double minPower, double maxPower, double powerStep, DataCollector dataCollector, boolean resetDataCollector, double minVelocity, double maxVelocity, double maxCycleVelocityChange, Subsystem ... subSystem) {
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.powerStep = powerStep;
        this.dataCollector = dataCollector;
        this.setPower = setPower;
        this.resetDataCollector = resetDataCollector;
        this.maxVelocity = maxVelocity;
        this.minVelocity = minVelocity;
        this.maxCycleVelocityChange = maxCycleVelocityChange;
        if(subSystem != null)
            addRequirements(subSystem);
    }

    @Override
    public void initialize() {
        // System.out.println(" sysid-powercycle-starting " + power);
        if(resetDataCollector) {
            dataCollector.resetData();
        }
        power = minPower;
        setPower.accept(power);
        init = true;
        steadyCount = 0;
        lastV = 0;
    }

    @Override
    public void execute() {
        if(init) { // first time - does not collect, set the initial v in data collector
            dataCollector.resetLastV();
            init = false;
        } else {
            double v = dataCollector.getVelocity.get();  // the currect velocity
            if(Math.abs(v-lastV) < maxCycleVelocityChange) { // small velocity change
                steadyCount++;   // increment the count
            } else {
                steadyCount = 0; // big velocity change - reset the count
            }
            if(steadyCount > 5) { // steady state
                if(v > minVelocity) { // only record velocities above the minimum
                    dataCollector.lastV = v; // set the lastV so no acceleration
                    dataCollector.collect(power);
                }
                // move to next power
                power += powerStep;
                steadyCount = 0;
                setPower.accept(power);
            }
            lastV = v;
        }
    }

    @Override
    public boolean isFinished() {
        return power > maxPower || lastV > maxVelocity || power > 1;
    }

    @Override
    public void end(boolean interrupted) {
        setPower.accept(0.0);
    }
    
}