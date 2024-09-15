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


        tmp.put("tag_1.0",new Translation2d(inchToMeter(593.68), inchToMeter(9.68)));
        tmp.put("tag_2.0",new Translation2d(inchToMeter(637.21), inchToMeter(34.79)));
        tmp.put("tag_3.0",new Translation2d(inchToMeter(652.73), inchToMeter(196.17)));
        tmp.put("tag_4.0",new Translation2d(inchToMeter(652.73), inchToMeter(218.42)));
        tmp.put("tag_5.0",new Translation2d(inchToMeter(578.77), inchToMeter(323.00)));
        tmp.put("tag_6.0",new Translation2d(inchToMeter(72.5), inchToMeter(323.00)));
        tmp.put("tag_7.0",new Translation2d(inchToMeter(-1.50), inchToMeter(218.42)));
        tmp.put("tag_8.0",new Translation2d(inchToMeter(-1.50), inchToMeter(196.17)));
        tmp.put("tag_9.0",new Translation2d(inchToMeter(14.02), inchToMeter(34.79)));
        tmp.put("tag_10.0",new Translation2d(inchToMeter(57.54), inchToMeter(9.68)));
        tmp.put("tag_11.0",new Translation2d(inchToMeter(468.69), inchToMeter(146.19)));
        tmp.put("tag_12.0",new Translation2d(inchToMeter(468.69), inchToMeter(177.10)));
        tmp.put("tag_13.0",new Translation2d(inchToMeter(441.74), inchToMeter(161.62)));
        tmp.put("tag_14.0",new Translation2d(inchToMeter(209.48), inchToMeter(161.62)));
        tmp.put("tag_15.0",new Translation2d(inchToMeter(182.73), inchToMeter(177.10)));
        tmp.put("tag_16.0",new Translation2d(inchToMeter(182.73),inchToMeter(146.19)));

        //if bottom left is (0,0)
        tmp.put("red_stage_leg_mid",new Translation2d(inchToMeter(121),inchToMeter(161.64)));
        tmp.put("red_stage_leg_bottom",new Translation2d(inchToMeter(231.2),inchToMeter(97.64)));
        tmp.put("red_stage_leg_top",new Translation2d(inchToMeter(231.2),inchToMeter(220.26)));
        tmp.put("blue_stage_leg_mid",new Translation2d(inchToMeter(532.22),inchToMeter(161.64)));
        tmp.put("blue_stage_leg_bottom",new Translation2d(inchToMeter(422.02),inchToMeter(97.64)));
        tmp.put("blue_stage_leg_top",new Translation2d(inchToMeter(422.02),inchToMeter(220.26)));
        tmp.put("red_speaker_middle",new Translation2d(inchToMeter(36.68),inchToMeter(104.64)));
        tmp.put("blue_speaker_middle",new Translation2d(inchToMeter(616.52),inchToMeter(104.64)));

        //tmp.put("note", null);
        dic = Collections.unmodifiableMap(tmp);
    }
    

    public static double inchToMeter(double inch){
      return inch*0.0254;
    }
    public static final Map<Double,Double> HEIGHT_MAP;
    static{
  
      // the height are converted from iches to meters!!!!!!.
  
      // also the AprilTag heights is from the middle to floor of the AprilTag.
      double mid_Height = 1.355852;//group 1   *
      double hight_Height = 1.451102;// gorup 2   #
      double Low_Height =1.3208;// group 3 !
      
      HashMap<Double,Double> idToHeights = new HashMap<>();
  
      idToHeights.put(1.0,mid_Height);//   *
      idToHeights.put(2.0,mid_Height);//   *
  
      idToHeights.put(3.0,hight_Height);//  #
      idToHeights.put(4.0,hight_Height);//  #
  
      idToHeights.put(5.0,mid_Height);
      idToHeights.put(6.0,mid_Height);
  
  
      idToHeights.put(7.0,hight_Height);
      idToHeights.put(8.0,hight_Height);
  
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
    public static final double LimelightHight = 0.317;
    public static final double LimelightAngle = 33;

    public static final double LimelightXOfset = 0.08;
    public static final double LimelightYOfset = 0.36;

}//1.095
