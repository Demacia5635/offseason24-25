package frc.robot.utils;

import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class EntryData {
    private static DataLog log = DataLogManager.getLog();
    private static String root = "Robot/";
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable rootTable = instance.getTable(root);
    
    private DoubleLogEntry entry;
    private DoublePublisher publisher;
    private DoubleSupplier supplier;
    private LongSupplier timestamp;
    
    private double lastValue = 0;

    public EntryData(String name, DoubleSupplier supplier) {
        this.supplier = supplier;
        entry = new DoubleLogEntry(log, root + name);
        publisher = rootTable.getDoubleTopic(name).publish();
    }

    public EntryData(String name, DoubleSupplier supplier, LongSupplier timestamp) {
        this.supplier = supplier;
        entry = new DoubleLogEntry(log, root + name);
        publisher = rootTable.getDoubleTopic(name).publish();
        this.timestamp = timestamp;
        forceLog();
    }

    public void log() {
        double v = supplier.getAsDouble();
        if (v != lastValue) {
            if (timestamp != null)
                entry.append(v, timestamp.getAsLong());
            else
                entry.append(v);
            publisher.set(v);
            lastValue = v;
        }
    }
    
    public void forceLog() {
        double v = supplier.getAsDouble();
        if (timestamp != null)
            entry.append(v, timestamp.getAsLong());
        else
            entry.append(v);
        publisher.set(v);
        lastValue = v;
        lastValue = v;
    }
}