package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public abstract class LogEntry {
    private static final NetworkTableInstance INSTANCE = NetworkTableInstance.getDefault();
    private static final String ROOT = "Robot/";
    private static final NetworkTable ROOT_TABLE = INSTANCE.getTable(ROOT);
    private static final DataLog log = DataLogManager.getLog();

    public class EntryData {
        private DoubleLogEntry entry;
        private DoublePublisher publisher;
        private DoubleSupplier supplier;
        
        private double lastValue = 0;

        public EntryData(String name, DoubleSupplier supplier) {
            this.supplier = supplier;
            entry = new DoubleLogEntry(log, ROOT + name);
            publisher = ROOT_TABLE.getDoubleTopic(name).publish();
        }

        public void log() {
            double v = supplier.getAsDouble();
            if (v != lastValue) {
                entry.append(v);
                publisher.set(v);
                lastValue = v;
            }
        }
    }

    protected List<EntryData> data;

    protected LogEntry() {
        data = new ArrayList<>();
    }

    public void update() {
        for (EntryData entryData : data) {
            entryData.log();
        }
    }
}