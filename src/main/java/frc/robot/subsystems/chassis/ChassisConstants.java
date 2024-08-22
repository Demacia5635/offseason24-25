package frc.robot.subsystems.chassis;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.chassis.utils.SwerveKinematics;

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

  public static final double BACK_STEER_RATIO = 151.0 / 7.0;
  public static final double FRONT_STEER_RATIO = 12.8;

  public static final double FRONT_PULSES_PER_DEGREE = FRONT_STEER_RATIO * MOTOR_PULSES_PER_ROTATION / 360.0;
  public static final double BACK_PULSES_PER_DEGREE =  BACK_STEER_RATIO * MOTOR_PULSES_PER_ROTATION / 360.0;
  
  // PID
  public static final PID_Constants MOVE_PID = new PID_Constants(0.085598906233349*10*1023/PULSES_PER_METER, 0, 0);
  public static final PID_Constants FRONT_STEER_PID = new PID_Constants(0.000209225899609*10*1023/FRONT_PULSES_PER_DEGREE, 0, 0);
  public static final PID_Constants BACK_STEER_PID = new PID_Constants(0.001104748806054*10*1023/BACK_PULSES_PER_DEGREE, 0, 0.001071468046139);
  public static final double MOVE_KV2 = -0.059217884557999;
  public static final double MOVE_KVSQRT = -0.506283997180385;
  // Feed Forward Gains
  // public static final FF_Constants MOVE_FF = new FF_Constants(0.1496659759518384, 0.405476931680402, 0.02251759948634);
  public static final FF_Constants FRONT_STEER_FF = new FF_Constants(0.069108623637248, 0.00034365326824, 0.000702476229803);
  public static final FF_Constants BACK_STEER_FF = new FF_Constants(0.080821555555163, 0.000529165452406, 0.004994578577863);
  public static final FF_Constants MOVE_FF_LESS = new FF_Constants(0.1, 0.2, 0.02251759948634);
  public static final FF_Constants MOVE_FF_MORE = new FF_Constants(0.2, 0.223, 0.03251759948634);
  public static final FF_Constants MOVE_FF_MORE2 = new FF_Constants(1, 0.25, 0.03251759948634);


//left front
  public static final PID_Constants FRONT_POSITION_STEER_PID_LEFT = new PID_Constants(0.067, 0.002, 0.098);
  public static final FF_Constants FRONT_STEER_FF_LEFT = new FF_Constants(0.069108623637248, 0.00034365326824, 0.000702476229803);



  // public static final PID_Constants BACK_POSITION_STEER_PID = new PID_Constants(0.036894342949841, 0.003689434294984, 0.000368943429498);
  public static final PID_Constants FRONT_POSITION_STEER_PID = new PID_Constants(0.067, 0.002, 0.098);
  public static final PID_Constants BACK_POSITION_STEER_PID = new PID_Constants(0.055, 0.0015, 0.000098);
  public static final double FRONT_INTEGRAL_ZONE = 9;
  public static final double BACK_INTEGRAL_ZONE = 8;


//useful data
public static final double METER_IN_CM = 100;
public static final double COLLECT_OFFSET_METERS = 0.7;


  public final static SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants(
      4, 5, 6,
      new Translation2d(0.332, 0.277),
      (-0.354*360) - 90,
      MOVE_PID,
      FRONT_STEER_PID,
      FRONT_POSITION_STEER_PID,
      MOVE_FF_LESS,
      MOVE_FF_MORE,
      MOVE_FF_MORE2,

      FRONT_STEER_FF,
      PULSES_PER_METER,
      FRONT_PULSES_PER_DEGREE,
      false,
      FRONT_INTEGRAL_ZONE);
  public final static SwerveModuleConstants FRONT_RIGHT = new SwerveModuleConstants(
      1, 2, 3,
      new Translation2d(0.332, -0.277),
      (-0.297*360) - 90,
      MOVE_PID,
      FRONT_STEER_PID,
      FRONT_POSITION_STEER_PID,
      MOVE_FF_LESS,
      MOVE_FF_MORE,
      MOVE_FF_MORE2,

      FRONT_STEER_FF,
      PULSES_PER_METER,
      FRONT_PULSES_PER_DEGREE,
      false,
      FRONT_INTEGRAL_ZONE);

  public final static SwerveModuleConstants BACK_LEFT = new SwerveModuleConstants(
      10, 11, 12,
      new Translation2d(-0.332, 0.288),
      (-0.382*360) - 90,
      MOVE_PID,
      BACK_STEER_PID,
      BACK_POSITION_STEER_PID,
      MOVE_FF_LESS,
      MOVE_FF_MORE,
      MOVE_FF_MORE2,

      BACK_STEER_FF,
      PULSES_PER_METER,
      BACK_PULSES_PER_DEGREE,
      true,
      BACK_INTEGRAL_ZONE);

  public final static SwerveModuleConstants BACK_RIGHT = new SwerveModuleConstants(
      7, 8, 9,
      new Translation2d(-0.332, -0.288),
      (-0.277*360) - 90,
      MOVE_PID,
      BACK_STEER_PID,
      BACK_POSITION_STEER_PID,
      MOVE_FF_LESS,
      MOVE_FF_MORE,
      MOVE_FF_MORE2,
      BACK_STEER_FF,
      PULSES_PER_METER,
      BACK_PULSES_PER_DEGREE,
      true,
      BACK_INTEGRAL_ZONE);

  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      FRONT_LEFT.moduleTranslationOffset,
      FRONT_RIGHT.moduleTranslationOffset,
      BACK_LEFT.moduleTranslationOffset,
      BACK_RIGHT.moduleTranslationOffset);
  

  public static final SwerveKinematics KINEMATICS_CORRECTED = new SwerveKinematics(
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
    public final double INTEGRAL_ZONE;
    public final int moveMotorId;
    public final int angleMotorId;
    public final int absoluteEncoderId;
    public final Translation2d moduleTranslationOffset;
    public final double steerOffset;
    public final PID_Constants movePID;
    public final PID_Constants steerPID;
    public final PID_Constants steerPositionPID;
    public final FF_Constants moveFFSlow;
    public final FF_Constants moveFFFast;
    public final FF_Constants moveFFFast2;

    public final FF_Constants steerFF;
    public final double pulsePerMeter;
    public final double pulsePerDegree;
    public final boolean inverted;

    public SwerveModuleConstants(int moveMotorId, int angleMotorId, int absoluteEncoderId,
        Translation2d moduleTranslationOffset, double steerOffset,
        PID_Constants movePID, PID_Constants steerPID, PID_Constants steerPositionPID, FF_Constants moveFFLess, FF_Constants moveFFMore, FF_Constants moveFFmore2, FF_Constants steerFF,
        double pulsePerMeter, double pulsePerDegree, boolean inverted, double INTEGRAL_ZONE) {
      this.moveMotorId = moveMotorId;
      this.angleMotorId = angleMotorId;
      this.absoluteEncoderId = absoluteEncoderId;
      this.moduleTranslationOffset = moduleTranslationOffset;
      this.steerOffset = steerOffset;
      this.movePID = movePID;
      this.moveFFSlow = moveFFLess;
      this.moveFFFast = moveFFMore;
      this.moveFFFast2 = moveFFmore2;
      this.steerFF = steerFF;
      this.steerPID = steerPID;
      this.pulsePerDegree = pulsePerDegree;
      this.pulsePerMeter = pulsePerMeter;
      this.inverted = inverted;
      this.steerPositionPID = steerPositionPID;
      this.INTEGRAL_ZONE = INTEGRAL_ZONE;
    }
  }
}
