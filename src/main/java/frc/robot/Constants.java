// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collections;
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
}
