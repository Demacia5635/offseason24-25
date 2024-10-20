package frc.robot.chassis.subsystems;


import static frc.robot.chassis.ChassisConstants.*;

import java.util.Arrays;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.chassis.utils.DemaciaOdometry;
import frc.robot.utils.LogManager;
import frc.robot.vision.subsystem.VisionByNote;
import frc.robot.vision.subsystem.VisionByTag;



public class Chassis extends SubsystemBase {
  private final SwerveModule[] modules;
  public final Pigeon2 gyro;
  private SwerveDrivePoseEstimator poseEstimator;
  private DemaciaOdometry demaciaOdometry;
  private final Field2d field;
  private Pose2d mergedPose;
 
 

  public static double targetVelocity = 0;
  public static double currentVelocity = 0;

  public VisionByTag visionByTag;
  public VisionByNote visionByNote;
  private PIDController rotationPID = new PIDController(0.3,0.0, 0.00);
  private boolean hasCalibratedPose = false;

  private boolean isAutoIntake = false;

  public Chassis() {
    modules = new SwerveModule[] {
        new SwerveModule(FRONT_LEFT, this),
        new SwerveModule(FRONT_RIGHT, this),
        new SwerveModule(BACK_LEFT, this),
        new SwerveModule(BACK_RIGHT, this),
    };    
    gyro = new Pigeon2(GYRO_ID, Constants.CANBUS);
    gyro.setYaw(0);

    this.visionByTag = new VisionByTag(gyro);
    this.visionByNote = new VisionByNote(new Pose2d());
    
    poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getRawAngle(), getModulePositions(), new Pose2d());
    visionByNote = new VisionByNote(poseEstimator.getEstimatedPosition());
    demaciaOdometry = new DemaciaOdometry(getRawAngle(), getModulePositions(), getPose());
    setBrake(true);
    field = new Field2d();
    SmartDashboard.putData(field);
//    SmartDashboard.putData(this);
    

    ShuffleboardTab ntTab = Shuffleboard.getTab("Chassis");
    //var setGyroEntry = ntTab.add("Set Gyro Angle", 0).getEntry();
    ntTab.add("Set gyro to 0 ",
       new InstantCommand( () -> setGyroAngle(0))
       .ignoringDisable(true));

    ntTab.add("gyro",gyro.getAngle());

    ntTab.add("Set gyro to 180 ",
       new InstantCommand( () -> setGyroAngle(180))
       .ignoringDisable(true));


    ntTab.add("run command", 
      new RunCommand(()->{setModulesPower(1); setModulesAngleFromSB(0);}, this));

    ntTab.add("reset gyro", new InstantCommand(()-> setGyroAngle(0)));
    ntTab.add("set coast",
        new InstantCommand(() -> setBrake(false)).ignoringDisable(true));
    ntTab.add("set brake",
        new InstantCommand(() -> setBrake(true)).ignoringDisable(true));
    ntTab.add("reset wheels", new InstantCommand(() -> resetWheels()).ignoringDisable(true));
        SmartDashboard.putData("reset pose", new InstantCommand(() -> setOdometryToForward()).ignoringDisable(true));

    ntTab.add("calibrate", new InstantCommand(()->calibrate(), this).ignoringDisable(true));


    //   ntTab.add("Set Modules Angle", 
    //     new RunCommand(() -> setModulesAngleFromSB(0)).ignoringDisable(true));
    //  ntTab.add("SetAngle45", new RunCommand(() -> {
    //   Rotation2d angle = Rotation2d.fromDegrees(45);
    //   for (SwerveModule module : modules) {
    //     module.setSteerPosition(angle);  
    //   }});

    
    var e = ntTab.add("Set Steer Rot", 0.5).getEntry();
    ntTab.add("Set Steer CMD", new RunCommand(()->modules[1].setSteerPosition(
      e.getDouble(0)),this));
    
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::setPose, 
      this::getChassisSpeeds,
      this::setVelRobot, 
      new HolonomicPathFollowerConfig(
        MAX_DRIVE_VELOCITY, 
        DRIVE_BASE_RADIUS, 
        new ReplanningConfig(
          true, 
          true
          )
        ),
      this::isRed, 
      this
    );
  }

  public void setVelRobot(ChassisSpeeds speeds){
    SwerveModuleState[] states = KINEMATICS_DEMACIA.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_DRIVE_VELOCITY);
    targetVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).getNorm() ;
    currentVelocity = getVelocity().getNorm();
    
    setModuleStates(states);
  }

  public SwerveModule[] getModules() {
    return modules;
  }

  public void calibrate() {
    SmartDashboard.putNumber("LEFT FRONT", modules[0].getAngleRaw());
    SmartDashboard.putNumber("RIGHT FRONT", modules[1].getAngleRaw());
    SmartDashboard.putNumber("LEFT BACK", modules[2].getAngleRaw());
    SmartDashboard.putNumber("RIGHT BACK", modules[3].getAngleRaw());
  }

  public boolean isRed() {
    return RobotContainer.robotContainer.isRed();
  }

  public void setGyroAngle(double angle){
    Pose2d newPose = new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(angle));
    poseEstimator.resetPosition(getRawAngle(), getModulePositions(), newPose);
  }

  public SwerveModule getModule(int i) {
    return modules[i];
  }

  public void resetWheels() {
    Rotation2d angle = Rotation2d.fromDegrees(0);
    for (var module : modules) {
      module.setSteerPosition(angle.getRotations());
    }
  }

  /**
   * Stops the entire chassis
   */
  public void stop() {
    for (var m : modules) {
      m.stop();
    }
  }

  public void setModulesSteerPower(double power) {
    for (var m : modules) {
      m.setSteerPower(power);
    }
  }
  public void setModulesSteerPower(double power, int i) {

    modules[i].setSteerPower(power);
    
  }

  public void setModulesSteerVoltage(double Voltage) {
    for (var m : modules) {
      m.setSteerPower(Voltage);
    }
  }
  public void setModulesSteerVoltage(double Voltage, int i) {

    modules[i].setSteerVoltage(Voltage);
    
  }

  public void setModuleSteerVelocity(double velocity, int moduleIndex) {
    modules[moduleIndex].setSteerVelocity(velocity);
  }

  public void setModulesPower(double power) {
    for (var m : modules) {
      m.setPower(power);
      m.setSteerPower(0);
    }
  }



  public double[] getAngularVelocities() {
    double[] angularVelocities = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      angularVelocities[i] = modules[i].getSteerVelocity();
    }
    return angularVelocities;
  }

  public void setVelocity(double v) {
    for (SwerveModule m : modules) {
      m.setVelocity(v);
    }
  }

  public double[] getVelocities() {
    double[] velocities = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      velocities[i] = modules[i].getVelocity();
    }
    return velocities;
  }

  public void setPose(Pose2d pose){
    poseEstimator.resetPosition(getRawAngle(), getModulePositions(), pose);
  }


  public void setVelocitiesRobotRel(ChassisSpeeds speeds){
    SwerveModuleState[] states = KINEMATICS_DEMACIA.toSwerveModuleStates(speeds);

    setModuleStates(states);
  }

  /**
   * Sets the velocity of the chassis
   * 
   * @param speeds In m/s and rad/s
   */
  public void setVelocities(ChassisSpeeds speeds) {
    ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());

    //SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(relativeSpeeds);
    SwerveModuleState[] states = KINEMATICS_DEMACIA.toSwerveModuleStates(relativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_DRIVE_VELOCITY);
    targetVelocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).getNorm();
    currentVelocity = getVelocity().getNorm();
    
    setModuleStates(states);
  }


  public void setVelocitiesRotateToSpeaker(ChassisSpeeds speeds){
    Translation2d speaker = isRed() ? Field.RedSpeakerTarget : Field.SpeakerTarget;
    Rotation2d toSpeakerAngle = speaker.minus(getPose().getTranslation()).getAngle();
    speeds.omegaRadiansPerSecond = getOmegaToAngle(toSpeakerAngle);
    
    // System.out.println("WANTED ANGLE: " + toSpeakerAngle);
    // System.out.println("CUR ANGLE: " + getAngle());
    setVelocities(speeds);
    
  }

  public void setVelocitiesRotateToAngle(ChassisSpeeds speeds, Rotation2d angle){
    speeds.omegaRadiansPerSecond = getOmegaToAngle(angle);
    setVelocities(speeds);
  }

  public double getOmegaToAngle(Rotation2d fieldAngle){
    double kP = 0.4;
    double diffAngle = MathUtil.inputModulus(fieldAngle.minus(getAngle()).getDegrees(), -180,180);
    System.out.println("-------------------------------");
            System.out.println("wanted= " + fieldAngle.getDegrees() + " gyro=" + 
                    getAngle().getDegrees() + " diff=" + diffAngle);
    System.out.println("-------------------------------");
    return Math.abs(diffAngle) <= 1 ? 0: Math.toRadians(diffAngle * kP);
  }


  public Translation2d getVelocity() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public double getMoveVelocity() {
    return getVelocity().getNorm();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  /**
   * Sets the neutral mode of every motor in the chassis
   * 
   * @param mode
   */
  public void setBrake(boolean brake) {
    Arrays.stream(modules).forEach((module) -> module.setBrake(brake));
  }


  /**
   * Returns the angle of the gyro
   */
  public Rotation2d getRawAngle() {
    return Rotation2d.fromDegrees(-1*gyro.getAngle());
  }
  public Rotation2d getAngle() {
    return getPose().getRotation();
  }

  public void setOdometryToForward() {
    poseEstimator.resetPosition(getRawAngle(), getModulePositions(),
        new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(0)));
  }

  /**
   * Returns the position of every module
   * 
   * @return Position relative to the field
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] res = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      res[i] = modules[i].getModulePosition();
    }
    return res;
  }

  /**
   * Returns the state of every module
   * 
   * 
   * @return Velocity in m/s, angle in Rotation2d
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] res = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      res[i] = modules[i].getState();
    }
    return res;
  }


  
  public double getPoseX(){
    return getPose().getX();
  }
  public double getPoseY(){
    return getPose().getY();
  }

  /**
   * Sets the state of every module
   * 
   * @param states Velocity in m/s, angle in Rotation2d
   */
  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  public void setModulesSteerPosition(Rotation2d angle, int i) {
    modules[i].setSteerPosition(angle.getRotations());

  }

  public void setModulesSteervelocity(double speed, int i) {
    modules[i].setSteerVelocity(speed);
  }

  public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator(){
    return this.poseEstimator;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
//    builder.addDoubleProperty("velocity", () -> getVelocity().getNorm(), null);
//    builder.addDoubleProperty("Omega", () -> Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond), null);
//    builder.addDoubleProperty("Angle", () -> Utils.degrees(getAngle()), null);
    LogManager.addEntry("chassis/gyro", gyro::getYaw);
    LogManager.addEntry("chassis/poseX", this::getPoseX);
    LogManager.addEntry("chassis/poseY", this::getPoseY);
  }
  public void setModulesAngleFromSB(double angle) {;
    Rotation2d a = Rotation2d.fromDegrees(angle);
    for (SwerveModule module : modules) {
      module.setSteerPosition(a.getRotations());
    }
  }


  public double getGyroRate() {
    return gyro.getRate();
  }

  /*private Pose2d mergeVisionOdometry(Pose2d odometry, Pose2d vision){
    odometry = odometry.times(0.7);
    vision = vision.times(0.3);
    
   
    Pose2d result = new Pose2d(odometry.getX() + vision.getX(),
      odometry.getY() + vision.getY(),
      odometry.getRotation().plus(vision.getRotation()));

    return result;
  }*/


  public void setAutoIntake(boolean isAutoIntake) {
    this.isAutoIntake = isAutoIntake;  
  }
  public boolean isAutoIntake() {
    return isAutoIntake;
  }

  public void updateVisionPose(Pose2d pose) {
    poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - 0.05);
  }
  /*private Pose2d filterVisionWithOdometry(Pose2d vision, Pose2d odometry){
    double maxAngleDiff = 50;
    double maxDistanceDiff = 2;
    Pose2d diff = vision;
    if(vision.minus(odometry).getT >= )
  }*/

  @Override
  public void periodic() {

    poseEstimator.update(getRawAngle(), getModulePositions());
   
    field.setRobotPose(getPose().plus(new Transform2d(0, 0, new Rotation2d())));
 }
}