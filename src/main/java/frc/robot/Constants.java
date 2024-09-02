// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collections;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;

import frc.robot.utils.Point;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static final Map<String,Point> dic;
    static{
        Hashtable<String,Point> tmp = new Hashtable<String,Point>();
        //ימין אדום שמאל כחול☺
        //במטרים  
        //קצר : Y ארוך : X
        tmp.put("Top",new Point(9, 7));
        tmp.put("Left",new Point(2, 5));
        tmp.put("Bottom",new Point(2, 2));
        tmp.put("Note", null);
        dic = Collections.unmodifiableMap(tmp);
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
}
