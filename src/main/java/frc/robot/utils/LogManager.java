// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class LogManager extends SubsystemBase {

    public static LogManager logManager; // singelton reference

    private DataLog log; //
    private NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private NetworkTable table = ntInst.getTable("log");

    /*
     * class for a single data entry
     */
    public class LogEntry {
        DoubleLogEntry entry;  // wpilib log entry
        Supplier<StatusSignal<Double>> getterPhoenix6Status; // supplier of phoenix 6 status signal
        DoubleSupplier getterDouble; // supplier for double data - if no status signal provider
        BiConsumer<Double, Long> consumer = null; // optional consumer when data changed - data value and time
        String name;
        DoublePublisher ntPublisher;   // network table punlisher
        double lastValue = Double.MAX_VALUE; // last value - only logging when value changes

        /*
         * Constructor with the suppliers and boolean if add to network table
         */
        LogEntry(String name, Supplier<StatusSignal<Double>> getterPhoenix6Status, DoubleSupplier getterDouble,
                boolean addToNT) {

            this.entry = new DoubleLogEntry(log, name);
            this.getterPhoenix6Status = getterPhoenix6Status;
            this.getterDouble = getterDouble;
            this.name = name;
            if (addToNT) {
                DoubleTopic dt = table.getDoubleTopic(name);
                ntPublisher = dt.publish();
            } else {
                ntPublisher = null;
            }
        }

        /*
         * perform a periodic log
         * get the data from the getters and call the actual log
         */
        void log() {
            double v;
            long time = 0;

            if (getterPhoenix6Status != null) {
                var st = getterPhoenix6Status.get();
                if (st.getStatus() == StatusCode.OK) {
                    v = st.getValue();
                    time = (long) (st.getTimestamp().getTime() * 1000);
                } else {
                    v = 1000000 + st.getStatus().value;
                }
            } else {
                v = getterDouble.getAsDouble();
                time = 0;
            }
            log(v, time);
        }

        /*
         * log a value use zero (current) time
         */
        public void log(double v) {
            log(v, 0);
        }

        /*
         * Log data and time if data changed
         * also publish to network table (if required)
         * also call consumer if set
         */
        public void log(double v, long time) {
            if (v != lastValue) {
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

        // set the consumer
        public void setConsumer(BiConsumer<Double,Long> consumer) {
            this.consumer = consumer;
        }
    }

    // array of log entries
    ArrayList<LogEntry> logEntries = new ArrayList<>();

    // Log managerconstructor
    public LogManager() {
        logManager = this;
        DataLogManager.start();
        DataLogManager.logNetworkTables(false);
        log = DataLogManager.getLog();
        DriverStation.startDataLog(log);
    }

    /*
     * add a log entry with all data
     */
    private LogEntry add(String name, Supplier<StatusSignal<Double>> getterPhoenix, DoubleSupplier getterDouble,
            boolean addToNT) {
        LogEntry entry = new LogEntry(name, getterPhoenix, getterDouble, addToNT);
        logEntries.add(entry);
        return entry;
    }

    /*
     * get a log entry - if not found, creat one
     */
    private LogEntry get(String name, boolean addToNT) {
        LogEntry e = find(name);
        if(e != null) {
            return e;
        }
        return new LogEntry(name, null, null, addToNT);
    }

    /*
     * find a log entry by name
     */
    private LogEntry find(String name) {
        for(LogEntry e : logEntries) {
            if(name.equals(e.name)) {
                return e;
            }
        }
        return null;
    }

    /*
     * Static function - add log entry with all data
     */
    public static LogEntry addEntry(String name, Supplier<StatusSignal<Double>> getterPhoenix,
            DoubleSupplier getterDouble, boolean addToNT) {
        return logManager.add(name, getterPhoenix, getterDouble, addToNT);
    }
    /*
     * Static function - add log entry for status signal with option to add to network table
     */
    public static LogEntry addEntry(String name, Supplier<StatusSignal<Double>> getterPhoenix, boolean addToNT) {
        return logManager.add(name, getterPhoenix, null, addToNT);
    }
    /*
     * Static function - add log entry for double supplier with option to add to network table
     */
    public static LogEntry addEntry(String name, DoubleSupplier getterDouble, boolean addToNT) {
        return logManager.add(name, null, getterDouble, addToNT);
    }
    /*
     * Static function - add log entry for status signal with network table
     */
    public static LogEntry addEntry(String name, Supplier<StatusSignal<Double>> getterPhoenix) {
        return logManager.add(name, getterPhoenix, null, true);
    }
    /*
     * Static function - add log entry for double supplier with network table
     */
    public static LogEntry addEntry(String name, DoubleSupplier getterDouble) {
        return logManager.add(name, null, getterDouble, true);
    }
    /*
     * Static function - get an entry, create if not foune - will see network table is crating new
     */
    public static LogEntry getEntry(String name, boolean addToNT) {
        return logManager.get(name, addToNT);
    }
    /*
     * Static function - get an entry, create if not foune - will see network table is crating new
     */
    public static LogEntry getEntry(String name) {
        return logManager.get(name, true);
    }

    /*
     * Log text message - also will be sent System.out
     */
    public static void log(String message) {
        DataLogManager.log(message);
    }

    @Override
    public void periodic() {
        super.periodic();
        for (LogEntry e : logEntries) {
            e.log();
        }
    }
}