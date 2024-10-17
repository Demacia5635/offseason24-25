// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

/** Add your docs here. */
public class ShooterConstants {

  public static class MOTOR_IDS{
    
    public static final int MOTOR_UP_ID = 21;
    public static final int MOTOR_DOWN_ID = 22;
    public static final int MOTOR_FEEDING_ID = 20;
    public static final int ANGLE_CHANGING_ID = 30;

    public static final int LIMIT_SWITCH_ID = 0;

    public static final String CANBUS = "rio";
  }

  public static class SHOOTER_PID_FF{
    
    public static final double UP_MOTOR_KP = 0.0;
    public static final double UP_MOTOR_KI = 0.0;
    public static final double UP_MOTOR_KD = 0.0;
    public static final double UP_MOTOR_KS = 0.0576789961667519;
    public static final double UP_MOTOR_KV = 0.11715135567075642;
    public static final double UP_MOTOR_KA = 0.0044124397972312365;
    public static final double UP_MOTOR_KV2 = 7.229070730748346E-5;

    public static final double DOWN_MOTOR_KP = UP_MOTOR_KP;
    public static final double DOWN_MOTOR_KI = UP_MOTOR_KI;
    public static final double DOWN_MOTOR_KD = UP_MOTOR_KD;
    public static final double DOWN_MOTOR_KS = UP_MOTOR_KS;
    public static final double DOWN_MOTOR_KV = UP_MOTOR_KV;
    public static final double DOWN_MOTOR_KA = UP_MOTOR_KA;
    public static final double DOWN_MOTOR_KV2 = UP_MOTOR_KV2;

  }

  public static class SHOOTER_POW{
    public static final double FEEDING_MOTOR_POWER = 1;
    public static final double INTAKE_MOTOR_POWER = 1;

  }

  public static class SHOOTER_CONFIGS{
    public static final boolean IS_UP_MOTOR_INVERT = true;
    public static final boolean IS_DOWN_MOTOR_INVERT = false;
    public static final boolean IS_FEEDING_MOTOR_INVERT = false;

    public static final boolean IS_SHOOTING_MOTORS_BRAKE = false;
    public static final boolean IS_FEEDING_MOTOR_BRAKE = true;

    public static final double FREQHZ = 200;

  }

  public static class AMP_VAR{
    
      public static final double AMP_ANGLE = 46;
      public static final double MOTOR_UP_AMP_VELOCITY = 15;
      public static final double MOTOR_DOWN_AMP_VELOCITY = 25;
    
  }

  public static class STAGE_VAR{
    
      public static final double STAGE_ANGLE = 0;
      public static final double MOTOR_UP_STAGE_VELOCITY = 0;
      public static final double MOTOR_DOWN_STAGE_VELOCITY = 0;
    
  }

  public static class SUBWOFFER_VAR{
    
      public static final double SUBWOFFER_ANGLE = 52;
      public static final double MOTOR_UP_SUBWOFFER_VELOCITY = 50;
      public static final double MOTOR_DOWN_SUBWOFFER_VELOCITY = 50; 
  }

  public static class MAX_ERRORS{
    
      public static final double ANGLE_MAX_ERRORS = 0.5;
      public static final double UP_MOTOR_VEL_MAX_ERRORS = 0.5;
      public static final double DOWN_MOTOR_VEL_MAX_ERRORS = 0.5;
    
  }

  public static class ANGLE_CHANGING_PID_FF{
    
    public static final double KP = 0.21;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double KA = 0;
    
  }

  public static class ANGLE_CHANGING_CONFIGS{

    public static final double ANGLE_CHANGING_MAX_VELOCITY = 25;
    public static final double ANGLE_CHANGING_MAX_Acceleration  = 30;
    public static final double ANGLE_CHANGING_MAX_JERK = 0;

    public static final double ANGLE_VOLTAGE_RAMP = 0.7;

    public static final boolean IS_ANGLE_MOTOR_INVERT = true;
    
    public static final boolean IS_ANGLE_MOTORS_BRAKE = true;

    public static final double ANGLE_CHANGING_FREQHZ = 200;
    
    public static final double ANGLE_CHANGING_GEAR_RATIO = 1/2.0;

  }

  public static class ANGLE_CHANGING_POW{

      public static final double ANGLE_MOTOR_POWER = 0.1;

  }

  public static class ANGLE_CHANGING_VAR{

      public static final double A = 128;
      public static final double B = 128;
      public static final double C = 80;

      public static final double BASE_DIS = 25.12;
      public static final double TOP_ANGLE = 65;
      public static final double MIN_ANGLE = 35;

  }

  public static class ANGLE_CHANGING_CALIBRATION{

      public static final double UP_SPEED_CALIBRATION = 0.3;
      public static final double DOWN_SPEED_CALIBRATION = -0.3;

  }

  public static class DISTANCES{

      public static final double WING_DISTANCE = 231;
      public static final double RIVAL_WING_DISTANCE = 421;
  }

  public static class SHOOTER_ATRIBUTES {
    public static final double RADIOS = 0.36;
    public static final double SHOOTING_MOTORS_ROTATION_TO_METER = 2 * RADIOS * Math.PI;
    public static final double MIL_SEC_TO_SHOOT = 2000;

  }  

  /**
   * Lookup table in the format of: <br> </br>
   * {@code [dis, angle, upMotorVel, downMotorVel]}
   * <br></br>
   * <ul>
   *  <li> dis - distance from the wanted shooting place in meters (primary key) </li>
   *  <li> angle - the working angle in this position in degrees </li>
   *  <li> upMotorVel - the working up motor vel in this location in meter per seconds </li>
   *  <li> downMotorVel - the working down motor vel in this location in meters per seconds </li>
   * </ul>
   */
  public static class LOOKUP_TABLE_DATA {
    public static final double[][] DATA = {
      {1.5, 52, 50, 50}, {2.5, 42, 55, 55}
    };
  }

  public enum STATE{
    AMP, STAGE, SUBWOFFER, DELIVERY_MID, DELIVERY_RIVAL, SPEAKER, IDLE, TESTING;
  }
}