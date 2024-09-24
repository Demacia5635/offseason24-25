package frc.robot.utils.entries;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.LogEntry;

public class Translation2dLog extends LogEntry {
    public Translation2dLog(String name, Translation2d translation) {
        data.add(new EntryData(name + "/x", translation::getX));
        data.add(new EntryData(name + "/y", translation::getY));
        data.add(new EntryData(name + "/norm", translation::getNorm));
        data.add(new EntryData(name + "/angle", () -> translation.getAngle().getDegrees()));

    }
}
