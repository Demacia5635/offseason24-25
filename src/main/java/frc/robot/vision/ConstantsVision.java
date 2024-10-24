package frc.robot.vision;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class ConstantsVision {

  public static Translation2d[] ORIGON_TO_TAG = {null,//0
      new Translation2d(inchToMeter(593.68), inchToMeter(9.68)),//1
      new Translation2d(inchToMeter(637.21), inchToMeter(34.79)),//2
      new Translation2d(inchToMeter(652.73), inchToMeter(196.17)),//3
      new Translation2d(inchToMeter(652.73), inchToMeter(218.42)),//4
      new Translation2d(inchToMeter(578.77), inchToMeter(323.00)),//5
      new Translation2d(inchToMeter(72.5), inchToMeter(323.00)),//6
      new Translation2d(inchToMeter(-1.50), inchToMeter(218.42)),//7
      new Translation2d(inchToMeter(-1.50), inchToMeter(196.17)),//8
      new Translation2d(inchToMeter(14.02), inchToMeter(34.79)),//9
      new Translation2d(inchToMeter(57.54), inchToMeter(9.68)),//10
      new Translation2d(inchToMeter(468.69), inchToMeter(146.19)),//11
      new Translation2d(inchToMeter(468.69), inchToMeter(177.10)),//12
      new Translation2d(inchToMeter(441.74), inchToMeter(161.62)),//13
      new Translation2d(inchToMeter(209.48), inchToMeter(161.62)),//14
      new Translation2d(inchToMeter(182.73), inchToMeter(177.10)),//15
      new Translation2d(inchToMeter(182.73),inchToMeter(146.19))//16
  };

public static double[] TAG_ANGLE = {0,//0
      60.0,//1
      60.0,//2
      0.0,//3
      0.0,//4
      270.0,//5
      270.0,//6
      0.0,//7
      0.0,//8
      60.0,//9
      60.0,//10
      300.0,//11
      60.0,//12
      0.0,//13
      0.0,//14
      300.0,//15
      60.0//16
  };

  public static double inchToMeter(double inch){
    return inch*0.0254;
  }

  private static double mid_Height = 1.355852;
  private static double high_Height = 1.451102;
  private static double Low_Height =1.3208;

  public static double[] TAG_HIGHT = {0,//0
    mid_Height,//1
    mid_Height,//2
    high_Height,//3
    high_Height,//4
    mid_Height,//5
    mid_Height,//6
    high_Height,//7
    high_Height,//8
    mid_Height,//9
    mid_Height,//10
    Low_Height,//11
    Low_Height,//12
    Low_Height,//13
    Low_Height,//14
    Low_Height,//15
    Low_Height//16
  };

  // APRIL TAG
  public static final double TAG_LIMELIGHT_HEIGHT = 0.220;
  public static final double TAG_LIMELIGHT_ANGLE = 34;

  
  public static final double TAG_LIMELIGHT_X_OFEST = 0.23;
  public static final double TAG_LIMELIGHT_Y_OFEST = -0.14;

  public static final String TAG_TABLE = "limelight-tag";
  // NOTE
  public static final double NOTE_LIMELIGHT_HEIGHT = 0.211;
  public static final double NOTE_LIMELIGHT_ANGLE = 0;

  public static final double NOTE_LIMELIGHT_X_OFEST = -0.39345;
  public static final double NOTE_LIMELIGHT_Y_OFEST = 0.0;

  public static final String NOTE_TABLE = "limelight-note";

}