// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

/** Add your docs here. */
public class ShooterConstants {

  public static class MOTOR_IDS{
    
    public static final int MOTOR_UP_ID = 2;
    public static final int MOTOR_DOWN_ID = 35;
    public static final int MOTOR_FEEDING_ID = 4;
    public static final int ANGLE_CHANGING_ID = 5;

    public static final int LIMIT_SWITCH_ID = 0;

    public static final String CANBUS = "CANBUS";
  }


  public static class SHOOTER_PID_FF{
    
    public static final double KP = 0.5;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;

    public static final double KV2 = 0.0;

  }

    public static class SHOOTER_VEL{

      public static final double FEEDING_MOTOR_POWER = 0;

  }

  public static class SHOOTER_CONFIGS{
    public static final boolean IS_UP_MOTOR_INVERT = false;
    public static final boolean IS_DOWN_MOTOR_INVERT = true;
    public static final boolean IS_FEEDING_MOTOR_INVERT = false;

    public static final boolean IS_SHOOTING_MOTORS_BRAKE = false;
    public static final boolean IS_FEEDING_MOTOR_BRAKE = true;

    public static final double SHOOTER_FreqHz = 200;

  }

  public static class AMP_VAR{
    
      public static final double AMP_ANGLE = 0;
      public static final double MOTOR_UP_AMP_VELOCITY = 0;
      public static final double MOTOR_DOWN_AMP_VELOCITY = 0;
    
  }

  public static class STAGE_VAR{
    
      public static final double STAGE_ANGLE = 0;
      public static final double MOTOR_UP_STAGE_VELOCITY = 0;
      public static final double MOTOR_DOWN_STAGE_VELOCITY = 0;
    
  }

  public static class SUBWOFFER_VAR{
    
      public static final double SUBWOFFER_ANGLE = 0;
      public static final double MOTOR_UP_SUBWOFFER_VELOCITY = 0;
      public static final double MOTOR_DOWN_SUBWOFFER_VELOCITY = 0;
    
  }

  public static class ZONES{
    
      public static final double ANGLE_ZONE = 0;
      public static final double UP_MOTOR_VEL_ZONE = 0;
      public static final double DOWN_MOTOR_VEL_ZONE = 0;
    
  }

  public static class ANGLE_CHANGING_PID_FF{
    
    public static final double KP = 0.01;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 0.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    
  }

  public static class ANGLE_CHANGING_CONFIGS{

    public static final double ANGLE_CHANGING_MAX_VELOCITY = 10;
    public static final double ANGLE_CHANGING_MAX_Acceleration  = 15;
    public static final double ANGLE_CHANGING_MAX_JERK = 20;
    public static final double BASE_ANGLE = 0;

    public static final boolean IS_ANGLE_MOTOR_INVERT = false;
    
    public static final boolean IS_ANGLE_MOTORS_BRAKE = false;

    public static final double ANGLE_CHANGING_FreqHz = 200;

    public static final double ANGLE_CHANGING_GEAR_RATIO = 1/4;

  }


  public static class ANGLE_CHANGING_VAR{

      public static final double A = 128;
      public static final double B = 128;
      public static final double C = 0;

      public static final double BASE_ANGLE = 0;
      public static final double TOP_ANGLE = 0;
      public static final double MIN_ANGLE = 0;

  }

  public static class ANGLE_CHANGING_CALIBRATION{

      public static final double UP_SPEED_CALIBRATION = 0;
      public static final double DOWN_SPEED_CALIBRATION = 0;

  }

  public static class DISTANCES{

      public static final double WING_DISTANCE = 0;

  }

  public static final double MIL_SEC_TO_SHOOT = 1000;

  public enum STATE{
    AMP, STAGE, SUBWOFFER, DELIVERY_MID, DELIVERY_RIVAL, SPEAKER, IDLE, TESTING;
  }
}