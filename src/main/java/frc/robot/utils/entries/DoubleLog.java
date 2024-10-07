package frc.robot.utils.entries;

import java.util.function.DoubleSupplier;

import frc.robot.utils.LogEntry;

public class DoubleLog extends LogEntry {
    public DoubleLog(String name, DoubleSupplier supplier) {
        this.data.add(new EntryData(name, supplier));
    }
}
