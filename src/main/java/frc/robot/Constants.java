// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int Motor_UP_ID = 1;
  public static final int Motor_DOWN_ID = 2;
  public static final int MOTOR_FEEDING_ID = 3;
  public static final int ANALOG_INPUT_ID = 4;

  public static final String CANBUS = "canivore";

  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.0;
  public static final double kV = 0.0;

  //public static final double SCOPE = 
  public static final double RADIOS = 0.035;
  public static final double SPIN_PER_METER = (RADIOS*2)*Math.PI;//the gear ratio is 1 in motor up and motordown
  public static final double METER_PER_SPIN = 1/SPIN_PER_METER;

  public static final double FEEDING_MOTOR_POWER = 0;
  public static final double Motor_UP_VELOCITY = 0;
  public static final double Motor_DOWN_VELOCITY = 0;

  public static final double NOTE_VOLTAGE = 0;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
