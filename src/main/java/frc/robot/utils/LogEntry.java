package frc.robot.utils;

import java.util.HashMap;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.RobotController;

public abstract class LogEntry {
    protected String name;
    protected HashMap<String, DoubleSupplier> attributes;

    protected LogEntry(String name, int dataAmount) {
        this.name = name;
        attributes = new HashMap<>();
    }

    public void log() {
        for (Entry<String, DoubleSupplier> attrib : attributes.entrySet()) {
            double v = attrib.getValue().getAsDouble();
            double time = (long) (RobotController.getFPGATime() * 1000);
        
            log(v, time);
        }
    }

    public void log(double v) {
        log(v, 0);
    }

    public void log(double v, long time) {
        entry.append(v, time);
        if (ntPublisher != null) {
            ntPublisher.set(v);
        }
        if(consumer != null) {
            consumer.accept(v, time);
        }
        lastValue = v;
    }
}