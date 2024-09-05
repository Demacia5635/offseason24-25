// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
public class Constants {
    public static final Map<String,Translation2d> dic;
    static{
        Hashtable<String,Translation2d> tmp = new Hashtable<String,Translation2d>();
        //ימין אדום שמאל כחול☺
        //במטרים  
        //קצר : Y ארוך : X

        tmp.put("tag_1",new Translation2d(inchToMeter(593.68), inchToMeter(9.68)));
        tmp.put("tag_2",new Translation2d(inchToMeter(637.21), inchToMeter(34.79)));
        tmp.put("tag_3",new Translation2d(inchToMeter(652.73), inchToMeter(196.17)));
        tmp.put("tag_4",new Translation2d(inchToMeter(652.73), inchToMeter(218.42)));
        tmp.put("tag_5",new Translation2d(inchToMeter(578.77), inchToMeter(323.00)));
        tmp.put("tag_6",new Translation2d(inchToMeter(72.5), inchToMeter(323.00)));
        tmp.put("tag_7",new Translation2d(inchToMeter(-1.50), inchToMeter(218.42)));
        tmp.put("tag_8",new Translation2d(inchToMeter(-1.50), inchToMeter(196.17)));
        tmp.put("tag_9",new Translation2d(inchToMeter(14.02), inchToMeter(34.79)));
        tmp.put("tag_10",new Translation2d(inchToMeter(57.54), inchToMeter(9.68)));
        tmp.put("tag_11",new Translation2d(inchToMeter(468.69), inchToMeter(146.19)));
        tmp.put("tag_12",new Translation2d(inchToMeter(468.69), inchToMeter(177.10)));
        tmp.put("tag_13",new Translation2d(inchToMeter(441.74), inchToMeter(161.62)));
        tmp.put("tag_14",new Translation2d(inchToMeter(209.48), inchToMeter(161.62)));
        tmp.put("tag_15",new Translation2d(inchToMeter(182.73), inchToMeter(177.10)));
        tmp.put("tag_16",new Translation2d(inchToMeter(182.73),inchToMeter(146.19)));


        tmp.put("note", null);
        dic = Collections.unmodifiableMap(tmp);
    }

    public static double inchToMeter(double inch){
      return inch*0.0254;
    }
    public static final Map<Integer,Double> HEIGHT_MAP;
    static{
  
      // the height are converted from iches to meters!!!!!!.
  
      // also the AprilTag heights is from the middle to floor of the AprilTag.
      double ids_1_2_5_6_9_10_Height = 1.355852;//group 1   *
      double ids_3_4_7_8_Height = 1.451102;// gorup 2   #
      double ids_11_12_13_14_15_16_Height =1.3208;// group 3 !
      
      HashMap<Integer,Double> idToHeights = new HashMap<>();
  
      idToHeights.put(1,ids_1_2_5_6_9_10_Height);//   *
      idToHeights.put(2,ids_1_2_5_6_9_10_Height);//   *
  
      idToHeights.put(3,ids_3_4_7_8_Height);//  #
      idToHeights.put(4,ids_3_4_7_8_Height);//  #
  
      idToHeights.put(5,ids_1_2_5_6_9_10_Height);
      idToHeights.put(6,ids_1_2_5_6_9_10_Height);
  
  
      idToHeights.put(7,ids_3_4_7_8_Height);//  #
      idToHeights.put(8,ids_3_4_7_8_Height);//  #
  
      idToHeights.put(9,ids_1_2_5_6_9_10_Height);//   *
      idToHeights.put(10,ids_1_2_5_6_9_10_Height);//  *
  
      idToHeights.put(11,ids_11_12_13_14_15_16_Height);// !
      idToHeights.put(12,ids_11_12_13_14_15_16_Height);// !
      idToHeights.put(13,ids_11_12_13_14_15_16_Height);// !
      idToHeights.put(14,ids_11_12_13_14_15_16_Height);// !
      idToHeights.put(15,ids_11_12_13_14_15_16_Height);// !
      idToHeights.put(16,ids_11_12_13_14_15_16_Height);// !
  
      idToHeights.put(0,0.0);//note id is not known so it is 0.0
  
      HEIGHT_MAP = Collections.unmodifiableMap(idToHeights);
    }
    public static final double LimelightHight = 0.7;
    public static final double LimelightAngle = 30;
}
