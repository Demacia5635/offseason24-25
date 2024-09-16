package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.RobotController;

public abstract class LogEntry {
    private static final NetworkTableInstance INSTANCE = NetworkTableInstance.getDefault();
    private static final String ROOT = "/Robot/";
    private static final NetworkTable ROOT_TABLE = INSTANCE.getTable(ROOT);

    public class EntryData {
        private String name;
        private DoubleLogEntry entry;
        private DoubleTopic topic;
        private DoubleSupplier supplier;

        public EntryData(String name, DoubleSupplier supplier) {
            this.name = name;
            this.supplier = supplier;
        }

        public void log() {

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