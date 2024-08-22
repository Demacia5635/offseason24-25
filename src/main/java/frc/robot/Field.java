package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Field {

    public static final double INCH = 0.0254;
    public static final double FEET = 12*INCH;

    public static final double FiledLength = 54*FEET + 3.75*INCH;
    public static final double FiledWidth = 26*FEET + 11.25*INCH;

    public static final double CentrX = FiledLength/2;
    public static final double CentrY = FiledWidth/2;
    
    public static final Translation2d Centr = new Translation2d(CentrX, CentrY);
    public static final double StartingZoneWidth = 6*FEET + 4.125*INCH;
    public static final double WingWidth = 231.2*INCH;

    public static final double AMPZoneLength = 10*FEET + 10*INCH;
    public static final double AMPZoneWidth = 1*FEET + 5.75*INCH;
    public static final Translation2d AMPZoneCorner = new Translation2d(0, FiledWidth-AMPZoneWidth);
    public static final double AMPWidth = 48*INCH;
    public static final Translation2d AMP = new Translation2d(4*FEET + 1.5*INCH + AMPWidth/2 + 0.1, FiledWidth);
    public static final Translation2d SubShootPosition = new Translation2d(1.4,AMP.getY());
    public static final double ampx = 4*FEET + 1.5*INCH + AMPWidth/2;


    public static final Translation2d Podium = new Translation2d(121*INCH, CentrY);
    public static final Translation2d[] Stage = {
        new Translation2d(WingWidth-106.19*INCH,CentrY),
        new Translation2d(WingWidth,CentrY+61.31*INCH),
        new Translation2d(WingWidth,CentrY-61.31*INCH)
    };
    
    public static final double SourceZoneWidth = 1*FEET + 6.75*INCH;

    public static final double WingNoteX = 114*INCH;
    public static final Translation2d[] WingNotes = {
        new Translation2d(WingNoteX,CentrY + 2*57*INCH),
        new Translation2d(WingNoteX,CentrY+57*INCH),
        new Translation2d(WingNoteX,CentrY)
    };
    public static final Translation2d[] CenterNotes = {
        new Translation2d(CentrX,CentrY + 2*1.68),
        new Translation2d(CentrX,CentrY + 1*1.68),
        new Translation2d(CentrX,CentrY + 0*1.68),
        new Translation2d(CentrX,CentrY - 1*1.68),
        new Translation2d(CentrX,CentrY - 2*1.68)
    };

    public static final Translation2d Speaker = new Translation2d(-0.04, 5.55);
    public static final Translation2d SpeakerTarget = new Translation2d(0.1, 5.55);
    
    public static final double SubwooferWidth = 3*FEET + 0.125*INCH;

    public static Translation2d toRed(Translation2d blue) {
        
        return new Translation2d(FiledLength-blue.getX(), blue.getY());
    }

    public static final Rotation2d RedBaseRotation = new Rotation2d(Math.PI);
    public static Rotation2d toRed(Rotation2d blue) {
        return RedBaseRotation.minus(blue);
    }
    public static Pose2d toRed(Pose2d blue) {
        return new Pose2d(toRed(blue.getTranslation()), toRed(blue.getRotation()));
    }
    public static Translation2d[] toRed(Translation2d[] blue) {
        Translation2d[] red = new Translation2d[blue.length];
        for(int i = 0; i < blue.length; i++) {
            red[i] = toRed(blue[i]);
        }
        return red;
    }

    public static final Translation2d RedSpeaker = toRed(Speaker);
    public static final Translation2d RedSpeakerTarget = toRed(SpeakerTarget);
    public static final Translation2d RedAMP = toRed(AMP);

    public static final Translation2d RedSubShootPosition =toRed(SubShootPosition);

    public static Translation2d offset(Translation2d base, double dx, double dy) {
        return base.minus(new  Translation2d(dx,dy));
    }

}
