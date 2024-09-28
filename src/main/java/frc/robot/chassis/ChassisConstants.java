package frc.robot.chassis;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.utils.TalonConfig;

public final class ChassisConstants {
  public static final int GYRO_ID = 14;

  public static final double CYCLE_DT = 0.02;
  
  public static final double MAX_DRIVE_VELOCITY = 4.1;
  public static final double DRIVE_ACCELERATION = 50;
  public static final double MAX_STEER_VELOCITY = 600;
  public static final double STEER_ACCELERATION = 6000;
  public static final double MAX_STEER_ERROR = 1;
  public static final double MAX_OMEGA_VELOCITY = Math.toRadians(720);

  public static final double MAX_OMEGA_ACCELERATION = Math.toRadians(4000);

  // Pulse per meter/degrees
  public static final double WHEEL_DIAMETER = 4 * 0.0254; // 4 inch
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  public static final double MOVE_GEAR_RATIO = 8.14;
  public static final double MOTOR_PULSES_PER_ROTATION = 2048;
  public static final double PULSES_PER_METER = MOTOR_PULSES_PER_ROTATION * MOVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE;

  public static final double FRONT_STEER_RATIO = 151.0 / 7.0;
  public static final double BACK_STEER_RATIO = 12.8;

  public static final double FRONT_PULSES_PER_DEGREE = FRONT_STEER_RATIO * MOTOR_PULSES_PER_ROTATION / 360.0;
  public static final double BACK_PULSES_PER_DEGREE =  BACK_STEER_RATIO * MOTOR_PULSES_PER_ROTATION / 360.0;

  // PID
  public static final PID_Constants MOVE_PID = new PID_Constants(0.0000001, 0, 0);
  public static final PID_Constants FRONT_STEER_PID = new PID_Constants(0.1, 0.04, 0.0007);
  public static final PID_Constants BACK_STEER_PID = new PID_Constants(0.07, 0.0, 0.0);
  /*TODO kill who forgot to finish the merge */
  // public static final PID_Constants FRONT_STEER_PID = new PID_Constants(0.95, 0.009, 0.0001);
  // public static final PID_Constants BACK_STEER_PID = new PID_Constants(0.7, 0, 0.0001);
  public static final double MOVE_KV2 = -0.059217884557999;
  public static final double MOVE_KVSQRT = -0.506283997180385;
  // Feed Forward Gains
  // public static final FF_Constants MOVE_FF = new FF_Constants(0.1496659759518384, 0.405476931680402, 0.02251759948634);
  public static final FF_Constants FRONT_STEER_FF = new FF_Constants(0.03, 0.002, 0.001); //0.0262, 0.0008, 0.0004);
  public static final FF_Constants BACK_STEER_FF = new FF_Constants(0.02, 0.0, 0.0);
  public static final FF_Constants MOVE_FF_MORE = new FF_Constants(0.4, 2.7, 0.03251759948634);



  public static final double FRONT_INTEGRAL_ZONE = 9;
  public static final double BACK_INTEGRAL_ZONE = 8;


//useful data
public static final double METER_IN_CM = 100;
public static final double COLLECT_OFFSET_METERS = 0.7;

public static final double SICLE_CAUNT = 50.0;


  public final static SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants(
      "FrontLeft",
      new TalonConfig(4,"rio", "FrontLeft/Drive")
        .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD,MOVE_FF_MORE.KS,MOVE_FF_MORE.KV,MOVE_FF_MORE.KA,0)
        .withInvert(true).withMotorRatio(1/MOVE_GEAR_RATIO),
      new TalonConfig(5,"rio", "FrontLeft/Steer")
        .withPID(FRONT_STEER_PID.KP, FRONT_STEER_PID.KI, FRONT_STEER_PID.KD,FRONT_STEER_FF.KS,FRONT_STEER_FF.KV,FRONT_STEER_FF.KA,0)
        .withInvert(false).withMotorRatio(1/FRONT_STEER_RATIO)
        .withMotionMagic(500, 1440, 3000),
      6,
      new Translation2d(0.332, 0.277),
      -0.39599609375);

  public final static SwerveModuleConstants FRONT_RIGHT = new SwerveModuleConstants(

      "FrontRight",
      new TalonConfig(1,"rio", "FrontRight/Drive")
        .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD,MOVE_FF_MORE.KS,MOVE_FF_MORE.KV,MOVE_FF_MORE.KA,0)
        .withInvert(true).withMotorRatio(1/MOVE_GEAR_RATIO),
      new TalonConfig(2,"rio", "FrontRight/Steer")
        .withPID(FRONT_STEER_PID.KP, FRONT_STEER_PID.KI, FRONT_STEER_PID.KD,FRONT_STEER_FF.KS,FRONT_STEER_FF.KV,FRONT_STEER_FF.KA,0)
        .withInvert(false).withMotorRatio(1/FRONT_STEER_RATIO)
        .withMotionMagic(500, 1440, 3000),
      3,
      new Translation2d(0.332, -0.277),
      -0.448974609375);

  public final static SwerveModuleConstants BACK_LEFT = new SwerveModuleConstants(

    "BackLeft",
      new TalonConfig(10,"rio", "BackLeft/Drive")
        .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD,MOVE_FF_MORE.KS,MOVE_FF_MORE.KV,MOVE_FF_MORE.KA,0)
        .withInvert(true).withMotorRatio(1/MOVE_GEAR_RATIO),
      new TalonConfig(11,"rio", "BackLeft/Steer")
        .withPID(BACK_STEER_PID.KP, BACK_STEER_PID.KI, BACK_STEER_PID.KD,BACK_STEER_FF.KS,BACK_STEER_FF.KV,BACK_STEER_FF.KA,0)
        .withInvert(true).withMotorRatio(1/BACK_STEER_RATIO)
        .withMotionMagic(500, 1440, 3000),
      12,
      new Translation2d(-0.332, 0.288),
      -0.1357421875);

  public final static SwerveModuleConstants BACK_RIGHT = new SwerveModuleConstants(

      "BackRight",
      new TalonConfig(7,"rio", "BackRight/Drive")
        .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD,MOVE_FF_MORE.KS,MOVE_FF_MORE.KV,MOVE_FF_MORE.KA,0)
        .withInvert(true).withMotorRatio(1/MOVE_GEAR_RATIO),
      new TalonConfig(8,"rio", "BackRight/Steer")
        .withPID(BACK_STEER_PID.KP, BACK_STEER_PID.KI, BACK_STEER_PID.KD,BACK_STEER_FF.KS,BACK_STEER_FF.KV,BACK_STEER_FF.KA,0)
        .withInvert(true).withMotorRatio(1/BACK_STEER_RATIO)
        .withMotionMagic(500, 1440, 3000),
      9,
      new Translation2d(-0.332, -0.288),
      -0.48095703125);

      

  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      FRONT_LEFT.moduleTranslationOffset,
      FRONT_RIGHT.moduleTranslationOffset,
      BACK_LEFT.moduleTranslationOffset,
      BACK_RIGHT.moduleTranslationOffset);
  



  public static class PID_Constants {
    public final double KP, KI, KD;

    PID_Constants(double KP, double KI, double KD) {
      this.KP = KP;
      this.KI = KI;
      this.KD = KD;
    }
  }

  public static class FF_Constants {
    public final double KS, KV, KA;

    FF_Constants(double KS, double KV, double KA) {
      this.KS = KS;
      this.KV = KV;
      this.KA = KA;
    }
  }

  public static class SwerveModuleConstants {
    String name;
    public TalonConfig driveConfig;
    public TalonConfig steerConfig;
    public final int absoluteEncoderId;
    public final Translation2d moduleTranslationOffset;
    public final double steerOffset;

    public SwerveModuleConstants(String name, TalonConfig drive, TalonConfig steer, int cancoderID, 
      Translation2d offset, double steerOffset) {
        this.name = name;
        this.driveConfig = drive;
        this.steerConfig = steer;
        this.absoluteEncoderId = cancoderID;
        this.moduleTranslationOffset = offset;
        this.steerOffset = steerOffset;
      }
  }
}
