package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LogEntry {
    private static final NetworkTableInstance INSTANCE = NetworkTableInstance.getDefault();
    private static final String ROOT = "Robot/";
    private static final NetworkTable ROOT_TABLE = INSTANCE.getTable(ROOT);
    private static final DataLog log = DataLogManager.getLog();

    public class EntryData {
        private String name;
        private DoubleLogEntry entry;
        private DoubleTopic topic;
        private DoubleSupplier supplier;
        
        private double lastValue = 0;

        public EntryData(String name, DoubleSupplier supplier) {
            this.name = name;
            this.supplier = supplier;
            this.entry = new DoubleLogEntry(log, ROOT + name);
            topic = ROOT_TABLE.getDoubleTopic(name);
        }

        public void log() {
            double v = supplier.getAsDouble();
            if (v != lastValue) {
                entry.append(v);
                lastValue = v;
            }
        }
    }

    protected List<EntryData> data;

    public LogEntry() {
        data = new ArrayList<>();
    }

    public void addData(String name, DoubleSupplier getter) {
        EntryData ed = new EntryData(name, getter);
        this.data.add(ed);
    }

    public void update() {
        for (EntryData entryData : data) {
            entryData.log();
        }
    }
}