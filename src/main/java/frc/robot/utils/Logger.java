package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Logger {
    private static List<EntryData> logs = new ArrayList<>();

    public static void add(EntryData entry) {
        logs.add(entry);
    }
    
    public static void addDouble(String name, DoubleSupplier entry) {
        logs.add(new EntryData(name, entry));
    }

    public static void addRotation2d(String name, Supplier<Rotation2d> entry) {
        addDouble(name + "/degrees", entry.get()::getDegrees);
        addDouble(name + "/radians", entry.get()::getRadians);
        addDouble(name + "/rotations", entry.get()::getRotations);
        addDouble(name + "/cos", entry.get()::getCos);
        addDouble(name + "/sin", entry.get()::getSin);
        addDouble(name + "/tan", entry.get()::getTan);
    }

    public static void addTranslation2d(String name, Supplier<Translation2d> entry) {
        addDouble(name + "/x", entry.get()::getX);
        addDouble(name + "/y", entry.get()::getY);
        addRotation2d(name + "/angle", entry.get()::getAngle);
        addDouble(name + "/norm", entry.get()::getNorm);
    }

    public static void addPose2d(String name, Supplier<Pose2d> entry) {
        addTranslation2d(name + "/translation", entry.get()::getTranslation);
        addRotation2d(name + "/rotation", entry.get()::getRotation);
    }

    public static void addStatusSignal(String name, Supplier<StatusSignal<Double>> entry) {
        logs.add(new EntryData(
            name,
            entry.get().getValue()::doubleValue,
            () -> (long) entry.get().getTimestamp().getTime() * 1000
        ));
    }

    public static void update() {
        for (EntryData entry : logs) {
            entry.log();
        }
    }
}
