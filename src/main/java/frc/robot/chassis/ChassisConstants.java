package frc.robot.chassis;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.chassis.utils.SwerveKinematics;
import frc.robot.utils.TalonConfig;
import frc.robot.Constants;

public final class ChassisConstants {

  private static final String CANBAS = Constants.CANBUS;
  public static final int GYRO_ID = 14;

  public static final double CYCLE_DT = 0.02;
  
  public static final double MAX_DRIVE_VELOCITY = 4.1;
  public static final double DRIVE_ACCELERATION = 50;
  public static final double MAX_STEER_VELOCITY = 1.5; //in 
  public static final double MAX_STEER_POWER = 0.7;
  public static final double MAX_STEER_ACCELERATION = 3; // in RPS Squared
  public static final Rotation2d MAX_STEER_ERROR = Rotation2d.fromDegrees(1);
  public static final double MAX_OMEGA_VELOCITY = Math.toRadians(360);

  public static final double MAX_OMEGA_ACCELERATION = Math.toRadians(720);

  // Pulse per meter/degrees
  public static final double WHEEL_DIAMETER = 4 * 0.0254; // 4 inch
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  public static final double MOVE_GEAR_RATIO = 8.14;
  public static final double MOTOR_PULSES_PER_ROTATION = 2048;
  public static final double PULSES_PER_METER = MOTOR_PULSES_PER_ROTATION * MOVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE;

  public static final double STEER_RATIO = 12.8;


  // PID
  public static final PID_Constants MOVE_PID = new PID_Constants(0.0000001, 0, 0);
  public static final PID_Constants STEER_PID = new PID_Constants(29, /*1.57*/ 0.7 , 0.07);//(0.0025, 0.000004, 0.000022);
  public static final double MOVE_KV2 = -0.059217884557999;
  public static final double MOVE_KVSQRT = -0.506283997180385;
  // Feed Forward Gains
  // public static final FF_Constants MOVE_FF = new FF_Constants(0.1496659759518384, 0.405476931680402, 0.02251759948634);
  public static final FF_Constants STEER_FF = new FF_Constants(0.2742838015,1.51359078,  0.001411548535);
  public static final FF_Constants MOVE_FF_MORE = new FF_Constants(0.4, 2.7, 0.03251759948634);



  public static final double INTEGRAL_ZONE = 8;


//useful data
public static final double METER_IN_CM = 100;
public static final double COLLECT_OFFSET_METERS = 0.7;

public static final double SICLE_CAUNT = 50.0;

  public final static SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants(
      "FrontLeft",
      new TalonConfig(4,CANBAS, "FrontLeft/Drive")
        .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD,MOVE_FF_MORE.KS,MOVE_FF_MORE.KV,MOVE_FF_MORE.KA,0)
      .withInvert(false).withMotorRatio(MOVE_GEAR_RATIO),
      new TalonConfig(5,CANBAS, "FrontLeft/Steer")
        .withPID(STEER_PID.KP, STEER_PID.KI, STEER_PID.KD, STEER_FF.KS,STEER_FF.KV,STEER_FF.KA,0)
        .withInvert(true).withMotorRatio(STEER_RATIO),
      6,
      new Translation2d(0.173, 0.277),
      0.395263671875);

  public final static SwerveModuleConstants FRONT_RIGHT = new SwerveModuleConstants(

      "FrontRight",
      new TalonConfig(1,CANBAS, "FrontRight/Drive")
        .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD,MOVE_FF_MORE.KS,MOVE_FF_MORE.KV,MOVE_FF_MORE.KA,0)
        .withInvert(false).withMotorRatio(MOVE_GEAR_RATIO),
      new TalonConfig(2,CANBAS, "FrontRight/Steer")
        .withPID(STEER_PID.KP, STEER_PID.KI, STEER_PID.KD,STEER_FF.KS,STEER_FF.KV,STEER_FF.KA,0)
        .withInvert(true).withMotorRatio(STEER_RATIO)
        .withMotionMagic(720, 1440, 3000),
      3,
      new Translation2d(0.173, -0.277),
      0.351806640625);

  public final static SwerveModuleConstants BACK_LEFT = new SwerveModuleConstants(

    "BackLeft",
      new TalonConfig(7,CANBAS, "BackLeft/Drive")
        .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD,MOVE_FF_MORE.KS,MOVE_FF_MORE.KV,MOVE_FF_MORE.KA,0)
        .withInvert(false).withMotorRatio(MOVE_GEAR_RATIO),
      new TalonConfig(8,CANBAS, "BackLeft/Steer")
        .withPID(STEER_PID.KP, STEER_PID.KI, STEER_PID.KD,STEER_FF.KS,STEER_FF.KV,STEER_FF.KA,0)
        .withInvert(true).withMotorRatio(STEER_RATIO)
        .withMotionMagic(720, 1440, 3000),
      9,
      new Translation2d(-0.313, 0.277),
      0.399658203125);

  public final static SwerveModuleConstants BACK_RIGHT = new SwerveModuleConstants(

      "BackRight",
      new TalonConfig(10,CANBAS, "BackRight/Drive")
        .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD,MOVE_FF_MORE.KS,MOVE_FF_MORE.KV,MOVE_FF_MORE.KA,0)
        .withInvert(false).withMotorRatio(MOVE_GEAR_RATIO),
      new TalonConfig(11,CANBAS, "BackRight/Steer")
        .withPID(STEER_PID.KP, STEER_PID.KI, STEER_PID.KD,STEER_FF.KS,STEER_FF.KV,STEER_FF.KA,0)
        .withInvert(true).withMotorRatio(STEER_RATIO)
        .withMotionMagic(720, 1440, 3000),
      12,
      new Translation2d(-0.332, -0.277),
      0.45361328125);

      

  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      FRONT_LEFT.moduleTranslationOffset,
      FRONT_RIGHT.moduleTranslationOffset,
      BACK_LEFT.moduleTranslationOffset,
      BACK_RIGHT.moduleTranslationOffset);
  

  public static final SwerveKinematics KINEMATICS_DEMACIA = new SwerveKinematics(
      FRONT_LEFT.moduleTranslationOffset,
      FRONT_RIGHT.moduleTranslationOffset,
      BACK_LEFT.moduleTranslationOffset,
      BACK_RIGHT.moduleTranslationOffset
  );


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
