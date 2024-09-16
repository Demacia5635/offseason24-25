// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

/** Add your docs here. */
public class ShooterConstants {
    public static final int MOTOR_UP_ID = 1;
  public static final int MOTOR_DOWN_ID = 2;
  public static final int MOTOR_FEEDING_ID = 3;
  public static final int ANALOG_INPUT_ID = 4;
  public static final int ANGLE_CHANGING_ID = 5;

  public static final String CANBUS = "canivore";

  public static final double SHOOTER_KP = 0.0;
  public static final double SHOOTER_KI = 0.0;
  public static final double SHOOTER_KD = 0.0;
  public static final double SHOOTER_KS = 0.0;
  public static final double SHOOTER_KV = 0.0;

public static final double ANGLE_CHANGING_KP = 0.0;
  public static final double ANGLE_CHANGING_KI = 0.0;
  public static final double ANGLE_CHANGING_KD = 0.0;
  public static final double ANGLE_CHANGING_KS = 0.0;
  public static final double ANGLE_CHANGING_KV = 0.0;
  public static final double ANGLE_CHANGING_KA = 0.0;

  //public static final double SCOPE = 
  public static final double RADIOS = 0.035;
  public static final double SPIN_PER_METER = (RADIOS*2)*Math.PI;//the gear ratio is 1 in motor up and motordown
  public static final double METER_PER_SPIN = 1/SPIN_PER_METER;
  public static final double ANGLE_CHANGING_GEAR_RATIO = 0;
  public static final double OOM_SPIN_PER_METER = 0;

  public static final double FEEDING_MOTOR_POWER = 0;
  public static final double Motor_UP_VELOCITY = 0;
  public static final double Motor_DOWN_VELOCITY = 0;
  public static final double CALIBRITION_ANGLE_CHAGING_VELOCITY = 0;
  public static final double UP_SPEED_CALIBRATION = 0;
  public static final double DOWN_SPEED_CALIBRATION = 0;
  
  public static final double ANGLE_CHANGING_MAX_VELOCITY = 0;
  public static final double ANGLE_CHANGING_MAX_Acceleration  = 0;
  public static final double ANGLE_CHANGING_MAX_Jerk = 0;

  public static final double SHOOOTER_VOLTAGE = 0;
}