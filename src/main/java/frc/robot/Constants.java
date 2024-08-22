package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  
    

  public static final double CYCLE_DT = 0.02;
  public static final int CONTROLLER_PORT = 0;

  public static class ChassisConstants {
    public final static SwerveModuleConstants MODULE_FRONT_LEFT = new SwerveModuleConstants(
      3, 4, 12,
      new Translation2d(0.332, 0.277),
      257.607421875,
      true
    );
    public final static SwerveModuleConstants MODULE_FRONT_RIGHT = new SwerveModuleConstants(
      5, 6, 13,
      new Translation2d(0.332, -0.277),
      290.7,
      true
    );
    public final static SwerveModuleConstants MODULE_BACK_LEFT = new SwerveModuleConstants(
      2, 1, 11,
      new Translation2d(-0.332, 0.288),
      230,
      false
    );
    public final static SwerveModuleConstants MODULE_BACK_RIGHT = new SwerveModuleConstants(
      8, 7, 14,
      new Translation2d(-0.332, -0.288),
      276.1,
      false
    );
    public static final int GYRO_ID = 15;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      MODULE_FRONT_LEFT.moduleTranslationOffset,
      MODULE_FRONT_RIGHT.moduleTranslationOffset,
      MODULE_BACK_LEFT.moduleTranslationOffset,
      MODULE_BACK_RIGHT.moduleTranslationOffset
    );

    public static final double MAX_DRIVE_VELOCITY = 4;
    public static final double DRIVE_ACCELERATION = 12;
    public static final double MAX_STEER_VELOCITY = 600;
    public static final double STEER_ACCELERATION = 6000;

    public static final double FORWORD_PULSES_PER_METER = 52226.56641604010025062656641604;
    public static final double FORWORD_PULSES_PER_DEGREE = (12.8 * 2048)/360;

    // public static final double WHEEL_DIAMETER = (4 * 2.54) / 100;
    // public static final double WHEEL_PERIMETER = WHEEL_DIAMETER * Math.PI;
    // public static final double MK4I_GEAR_RATIO = 8.14;
    // public static final double MOTOR_PULSES_PER_SPIN = 2048;
    // public static final double BACKWARD_PULSES_PER_METER = (1 / (WHEEL_PERIMETER))*MOTOR_PULSES_PER_SPIN * MK4I_GEAR_RATIO;
    // public static final double BACKWARD_PULSES_PER_DEGREE = 12.9047619048;
    public static final double BACKWARD_PULSES_PER_METER = 52226.56641604010025062656641604;
    public static final double BACKWARD_PULSES_PER_DEGREE = ((150/7)*2048)/360;

    public static class SwerveModuleConstants {
      public static final double FORWORD_MOVE_KP = 0.05;
      public static final double FORWORD_MOVE_KI = 0;
      public static final double FORWORD_MOVE_KD = 0;

      public static final double BACKWARD_MOVE_KP = 0.05;
      public static final double BACKWARD_MOVE_KI = 0;
      public static final double BACKWARD_MOVE_KD = 0;

      public static final double FORWORD_ANGLE_POSITION_KP = 0.001;
      public static final double FORWORD_ANGLE_POSITION_KI = 0.0000003;
      public static final double FORWORD_ANGLE_POSITION_KD = 0.0;
      public static final double FORWORD_ANGLE_VELOCITY_KP = 0.2;//0.05/*6.7422E-08*/; //0.07
      public static final double FORWORD_ANGLE_VELOCITY_KI = 0.0; //0.004;
      public static final double FORWORD_ANGLE_VELOCITY_KD = 0;

      public static final double BACKWARD_ANGLE_POSITION_KP = 0.0001;
      public static final double BACKWARD_ANGLE_POSITION_KI = 0.000001;
      public static final double BACKWARD_ANGLE_POSITION_KD = 0.0;
      public static final double BACKWARD_ANGLE_VELOCITY_KP = 0.2/*6.7422E-08*/; //0.07
      public static final double BACKWARD_ANGLE_VELOCITY_KI = 0.; //0.004;
      public static final double BACKWARD_ANGLE_VELOCITY_KD = 0;

      public static final double FORWORD_MOVE_KS = 0.02; // 0.15851/12; //0.0362;

      public static final double FORWORD_MOVE_KV = 0.1; //0.012314/12; //0.0862;

      public static final double FORWORD_MOVE_KA = 0.1; //0.012314/12; //0.0862;
      public static final double FORWORD_ANGLE_KS = 0.05;//0.52557/12.0; //0.05;
      public static final double FORWORD_ANGLE_KV = 0.0006;//0.003737/12.0; //0.0962;
      public static final double FORWORD_ANGLE_KA = 0.001;//0.003737/12.0; //0.0962;

      public static final double BACKWARD_MOVE_KS = 0.02; // 0.15851/12; //0.0362;

      public static final double BACKWARD_MOVE_KV = 0.1; //0.012314/12; //0.0862;

      public static final double BACKWARD_MOVE_KA = 0.1; //0.012314/12; //0.0862;
      public static final double BACKWARD_ANGLE_KS = 0.047;//0.52557/12.0; //0.05;
      public static final double BACKWARD_ANGLE_KV = 0.00056;//0.003737/12.0; //0.0962;
      public static final double BACKWARD_ANGLE_KA = 0.001;//0.003737/12.0; //0.0962;

      public static final double MAX_STEER_ERROR = 1;

      public final int moveMotorId;
      public final int angleMotorId;
      public final int absoluteEncoderId;
      public final Translation2d moduleTranslationOffset;
      public final double steerOffset;
      public final boolean front;

      public SwerveModuleConstants(int moveMotorId, int angleMotorId, int absoluteEncoderId, Translation2d moduleTranslationOffset, double steerOffset, boolean front) {
        this.front = front;
        this.moveMotorId = moveMotorId;
        this.angleMotorId = angleMotorId;
        this.absoluteEncoderId = absoluteEncoderId;
        this.moduleTranslationOffset = moduleTranslationOffset;
        this.steerOffset = steerOffset;
      }

    }
  }
  
  public static final class VisionConstants {

    public static final String AmpSideRaspberryName = "Amp_Side_Raspberry";
    public static final String ShooterSideRaspberryName = "Shooter_Side_Raspberry";


    public static final Transform3d robotCenterToAmpSideRaspberry = new Transform3d(0.21, -0.3, 0.49, new Rotation3d(0, Math.toRadians(40), 0));

    public static final Transform3d robotCenterToShooterSideRaspberry = new Transform3d(-0.33, -0.23, 0.26, new Rotation3d(0, Math.toRadians(45), 0));

    public static final double maxValidVelcity = 2.0; // m/s - ignoring vision data abve this velocity
    public static final double maxValidAngleDiff = 10.0; // degrees - ignoring vision data if vision heading is off by more than this value
    public static final double maxDistanceOfCameraFromAprilTag = 4; // meters - ignoring vision data if apriltag is farther than this value
 
  }


}
