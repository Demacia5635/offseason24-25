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
public class PowerCycleCommand extends Command {

    double power;                   // the power to use
    DataCollector dataCollector;    // data collector
    Consumer<Double> setPower;      // set power function
    boolean init;                   // if init
    boolean resetDataCollector;     // reset the data collector data for rerun of the same command group

    /**
     * default Constructor - does not reset the data collector
     * @param setPower
     * @param power
     * @param dataCollector
     * @param subSystem
     */
    public PowerCycleCommand(Consumer<Double> setPower, double power, DataCollector dataCollector, Subsystem ... subSystem) {
        this(setPower, power, dataCollector, false, subSystem);
    }

    /**
     * Constructor - can reset the data collector data in intialize
     * @param setPower
     * @param power
     * @param dataCollector
     * @param resetDataCollector
     * @param subSystem
     */
    public PowerCycleCommand(Consumer<Double> setPower, double power, DataCollector dataCollector, boolean resetDataCollector, Subsystem ... subSystem) {
        this.power = power;
        this.dataCollector = dataCollector;
        this.setPower = setPower;
        this.resetDataCollector = resetDataCollector;
        if(subSystem != null)
            addRequirements(subSystem);
    }

    @Override
    public void initialize() {
        // System.out.println(" sysid-powercycle-starting " + power);
        if(resetDataCollector) {
            dataCollector.resetData();
        }
        setPower.accept(power);
        init = true;
    }

    @Override
    public void execute() {
        if(init) { // first time - does not collect, set the initial v in data collector
            dataCollector.resetLastV();
            init = false;
        } else {
            dataCollector.collect(power);
        }
    }

    @Override
    public void end(boolean interrupted) {
       // System.out.println(" sysid-powercycle-end " + power);
        setPower.accept(0.0);
    }
    
}