// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double GEAR_RATIO = 8.14;
  public static final double R = 2.54*2/100;
  public static final double SCOPE = 2*R*Math.PI;
  public static final double METER_PER_TURN = SCOPE/GEAR_RATIO;
  public static final String CANBUS = "canivore";

  
  public static final double KS = 0.02;  //SmartDashboard.getNumber("KS", 0);
  public static final double KV = 0.1; //SmartDashboard.getNumber("KV", 0);
  public static final double BACKWARD_MOVE_KS = 0.02;
  public static final double BACKWARD_MOVE_KV = 0.1; 

  public static final double KP = 0.05;
  public static final double KI = KP/10;
  public static final double KD = KI/10;

  
  public static final double FORWORD_ANGLE_KS = 0.05;//0.52557/12.0; //0.05;
  public static final double FORWORD_ANGLE_KV = 0.0006;
  public static final double BACKWARD_ANGLE_KV = 0.00056;//0.003737/12.0; //0.0962;
  public static final double BACKWARD_ANGLE_KA = 0.001;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
