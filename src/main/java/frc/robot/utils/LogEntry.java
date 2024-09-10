package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.networktables.DoublePublisher;

public abstract class LogEntry {
    protected String name;
    protected ArrayList<DoublePublisher> publishers;

    protected LogEntry(String name, int dataAmount) {
        this.name = name;
        publishers = new ArrayList<>();
    }
}
