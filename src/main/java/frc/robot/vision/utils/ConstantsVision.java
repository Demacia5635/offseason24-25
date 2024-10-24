package frc.robot.vision.utils;

import java.util.Collections;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;

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

  public static Translation2d[] origonToTag = {null,
      new Translation2d(inchToMeter(593.68), inchToMeter(9.68)),
      new Translation2d(inchToMeter(637.21), inchToMeter(34.79)),
      new Translation2d(inchToMeter(652.73), inchToMeter(196.17)),
      new Translation2d(inchToMeter(652.73), inchToMeter(218.42)),
      new Translation2d(inchToMeter(578.77), inchToMeter(323.00)),
      new Translation2d(inchToMeter(72.5), inchToMeter(323.00)),
      new Translation2d(inchToMeter(-1.50), inchToMeter(218.42)),
      new Translation2d(inchToMeter(-1.50), inchToMeter(196.17)),
      new Translation2d(inchToMeter(14.02), inchToMeter(34.79)),
      new Translation2d(inchToMeter(57.54), inchToMeter(9.68)),
      new Translation2d(inchToMeter(468.69), inchToMeter(146.19)),
      new Translation2d(inchToMeter(468.69), inchToMeter(177.10)),
      new Translation2d(inchToMeter(441.74), inchToMeter(161.62)),
      new Translation2d(inchToMeter(209.48), inchToMeter(161.62)),
      new Translation2d(inchToMeter(182.73), inchToMeter(177.10)),
      new Translation2d(inchToMeter(182.73),inchToMeter(146.19))
  };
public static double[] tagAngle = {0,//0
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
public static boolean[] isTagRed = {false,//0
      true,//1
      true,//2
      true,//3
      true,//4
      true,//5
      false,//6
      false,//7
      false,//8
      false,//9
      false,//10
      false,//11
      false,//12
      true,//13
      false,//14
      true,//15
      true//16
  };
  public static final Map<String,Translation2d> CARTESIANVECTORS_MAP;
  static{
      Hashtable<String,Translation2d> vector = new Hashtable<String,Translation2d>();


      vector.put("tag_1.0",new Translation2d(inchToMeter(593.68), inchToMeter(9.68)));
      vector.put("tag_2.0",new Translation2d(inchToMeter(637.21), inchToMeter(34.79)));
      vector.put("tag_3.0",new Translation2d(inchToMeter(652.73), inchToMeter(196.17)));
      vector.put("tag_4.0",new Translation2d(inchToMeter(652.73), inchToMeter(218.42)));
      vector.put("tag_5.0",new Translation2d(inchToMeter(578.77), inchToMeter(323.00)));
      vector.put("tag_6.0",new Translation2d(inchToMeter(72.5), inchToMeter(323.00)));
      vector.put("tag_7.0",new Translation2d(inchToMeter(-1.50), inchToMeter(218.42)));
      vector.put("tag_8.0",new Translation2d(inchToMeter(-1.50), inchToMeter(196.17)));
      vector.put("tag_9.0",new Translation2d(inchToMeter(14.02), inchToMeter(34.79)));
      vector.put("tag_10.0",new Translation2d(inchToMeter(57.54), inchToMeter(9.68)));
      vector.put("tag_11.0",new Translation2d(inchToMeter(468.69), inchToMeter(146.19)));
      vector.put("tag_12.0",new Translation2d(inchToMeter(468.69), inchToMeter(177.10)));
      vector.put("tag_13.0",new Translation2d(inchToMeter(441.74), inchToMeter(161.62)));
      vector.put("tag_14.0",new Translation2d(inchToMeter(209.48), inchToMeter(161.62)));
      vector.put("tag_15.0",new Translation2d(inchToMeter(182.73), inchToMeter(177.10)));
      vector.put("tag_16.0",new Translation2d(inchToMeter(182.73),inchToMeter(146.19)));

      //if bottom left is (0,0)
      vector.put("red_stage_leg_mid",new Translation2d(inchToMeter(121),inchToMeter(161.64)));
      vector.put("red_stage_leg_bottom",new Translation2d(inchToMeter(231.2),inchToMeter(97.64)));
      vector.put("red_stage_leg_top",new Translation2d(inchToMeter(231.2),inchToMeter(220.26)));
      vector.put("blue_stage_leg_mid",new Translation2d(inchToMeter(532.22),inchToMeter(161.64)));
      vector.put("blue_stage_leg_bottom",new Translation2d(inchToMeter(422.02),inchToMeter(97.64)));
      vector.put("blue_stage_leg_top",new Translation2d(inchToMeter(422.02),inchToMeter(220.26)));
      vector.put("red_speaker_middle",new Translation2d(inchToMeter(36.68),inchToMeter(104.64)));
      vector.put("blue_speaker_middle",new Translation2d(inchToMeter(616.52),inchToMeter(104.64)));

      //tmp.put("note", null);
      CARTESIANVECTORS_MAP = Collections.unmodifiableMap(vector);
  }

  public static final Map<String,Double> IDTOANGLE_MAP;
  static{
      Hashtable<String,Double> angles = new Hashtable<String,Double>();


      angles.put("tag_1.0",120.0);
      angles.put("tag_2.0",120.0);
      angles.put("tag_3.0",180.0);
      angles.put("tag_4.0",180.0);
      angles.put("tag_5.0",270.0);
      angles.put("tag_6.0",270.0);
      angles.put("tag_7.0",0.0);
      angles.put("tag_8.0",0.0);
      angles.put("tag_9.0",90.0);
      angles.put("tag_10.0",60.0);
      angles.put("tag_11.0",300.0);
      angles.put("tag_12.0",60.0);
      angles.put("tag_13.0",180.0);
      angles.put("tag_14.0",0.0);
      angles.put("tag_15.0",120.0);
      angles.put("tag_16.0",240.0);
      



      angles.put("note", 0.0);
      IDTOANGLE_MAP = Collections.unmodifiableMap(angles);
  }
  

  public static double inchToMeter(double inch){
    return inch*0.0254;
  }
  public static final Map<Double,Double> HEIGHT_MAP;
  static{

    // the height are converted from iches to meters!!!!!!.

    // also the AprilTag heights is from the middle to floor of the AprilTag.
    double mid_Height = 1.355852;//group 1   *
    double high_Height = 1.451102;// gorup 2   #
    double Low_Height =1.3208;// group 3 !
    
    HashMap<Double,Double> idToHeights = new HashMap<>();

    idToHeights.put(1.0,mid_Height);//   *
    idToHeights.put(2.0,mid_Height);//   *

    idToHeights.put(3.0,high_Height);//  #
    idToHeights.put(4.0,high_Height);//  #

    idToHeights.put(5.0,mid_Height);
    idToHeights.put(6.0,mid_Height);


    idToHeights.put(7.0,high_Height);
    idToHeights.put(8.0,high_Height);

    idToHeights.put(9.0,mid_Height);
    idToHeights.put(10.0,mid_Height);

    idToHeights.put(11.0,Low_Height);
    idToHeights.put(12.0,Low_Height);
    idToHeights.put(13.0,Low_Height);
    idToHeights.put(14.0,Low_Height);
    idToHeights.put(15.0,Low_Height);
    idToHeights.put(16.0,Low_Height);

    idToHeights.put(-1.0,0.0);
    idToHeights.put(0.0,0.0);

    HEIGHT_MAP = Collections.unmodifiableMap(idToHeights);
  }
  // APRIL TAG CODE ROBOT
  public static final double TagLimelightHeight = 0.220;
  public static final double TagLimelightAngle = 34;

  
  public static final double TagLimelightXOfset = 0.23;
  public static final double TagLimelightYOfset = -0.14;

  public static final String TagTable = "limelight-tag";

  public static final double NOTE_LIMELIGHT_RES_X = 320;

  public static final double NOTE_LIMELIGHT_POV_X = 63.3;

  public static final double ANGLE_PER_PIX_X = (NOTE_LIMELIGHT_POV_X/NOTE_LIMELIGHT_RES_X);

  public static final double NOTE_RIDIUS = 0.175;

  public static final double NoteLimelightHight = 0.211;
  public static final double NoteLimelightAngle = 0;

  public static final double NoteLimelightXOfset = -0.39345;
  public static final double NoteLimelightYOfset = 0.0;

  public static final String NoteTable = "limelight-note";

}