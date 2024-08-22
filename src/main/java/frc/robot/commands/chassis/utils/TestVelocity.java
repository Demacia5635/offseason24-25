package frc.robot.commands.chassis.utils;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TestVelocity extends Command {

    Subsystem subsystm;
    String name;
    Consumer<Double> setVelocity;
    Supplier<Double> getVelocity;
    double targetVelocity;
    double treshHold;
    double minV;
    double maxV;
    double startV;
    boolean treshHoldReached;
    int nHighs;
    int nLows;
    int n;
    double startTime;
    double treshHoldTime;
    double acceleration;
    double maxError;
    double avgVelocity;

    public TestVelocity(String name, Consumer<Double> setVelocity, Supplier<Double> getVelocity, double treshHold, Subsystem subsystem) {
        this.name = "Test " + name + " Velocity";
        this.setVelocity = setVelocity;
        this.getVelocity = getVelocity;
        this.subsystm = subsystem;
        this.treshHold = treshHold;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        startV = getVelocity.get();
        treshHoldReached = false;
        nHighs = 0;
        nLows = 0;
        n = 0;
        treshHoldTime = 0;
        acceleration = 0;
        maxError = 0;
        avgVelocity = 0;
    }

    @Override
    public void execute() {
        setVelocity.accept(targetVelocity);
        double v = getVelocity.get();
        double time = Timer.getFPGATimestamp();
        if(!treshHoldReached) {
            if(time > startTime) {
                acceleration = (v-startV)/(time-startTime);
            }
            if(v > minV) {
                treshHoldReached = true;
                treshHoldTime = time;
            }
        }
        if(treshHoldReached) {
            n++;
            if(v < minV) {
                nLows++;
            } else if(v > maxV) {
                nHighs++;
            }
            avgVelocity += v;
            maxError = Math.max(maxError,Math.abs(v-targetVelocity));
        }
    }

    private void updateVelocity(double velocity) {
        targetVelocity = velocity;
        minV = velocity*(1-treshHold);
        maxV = velocity*(1+treshHold);
        // System.out.println(name + " target velocity=" + targetVelocity);
    }

    private int goodPercent() {
        if(n <= 0) {
            return 0;
        }
        double goodN = n - nHighs - nLows;
        return (int)(goodN*100/n);
    }


    @Override
    public void end(boolean interrupted) {
        setVelocity.accept(0.0);
    }
    
}
