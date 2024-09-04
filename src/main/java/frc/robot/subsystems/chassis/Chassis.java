package frc.robot.subsystems.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.PathFollow;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.Sysid.Sysid;
import frc.robot.Sysid.Sysid.Gains;
import frc.robot.commands.chassis.tests.DriveStraightLine;
import frc.robot.subsystems.chassis.utils.SwerveKinematics;
import frc.robot.subsystems.vision.utils.LimelightVisionUtils;
import frc.robot.subsystems.vision.utils.UpdatedPoseEstimatorClasses.SwerveDrivePoseEstimator;
import frc.robot.utils.Utils;
import static frc.robot.subsystems.chassis.ChassisConstants.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;

public class Chassis extends SubsystemBase {
  private final SwerveModule[] modules;
  private final Pigeon2 gyro;
  private double speakerAngleError;
  private PIDController angleSpeakerPID = new PIDController(0.05,0.0, 0.002);
  

  public static List<pathPoint> pointsForPathTeleop = new ArrayList<pathPoint>();
  public static List<pathPoint> pointsForAuto = new ArrayList<pathPoint>();

  private SwerveDrivePoseEstimator poseEstimator;


  private final Field2d field;

  boolean isAimingSpeaker = false;

  public static double targetVelocity = 0;
  public static double currentVelocity = 0;


  public Chassis() {
    modules = new SwerveModule[] {
        new SwerveModule(FRONT_LEFT, this),
        new SwerveModule(FRONT_RIGHT, this),
        new SwerveModule(BACK_LEFT, this),
        new SwerveModule(BACK_RIGHT, this),
    };    

    //configAllFactoryDefaut();

    gyro = new Pigeon2(GYRO_ID);
    gyro.setYaw(0);
    poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getRawAngle(), getModulePositions(), new Pose2d());
    
    setNeutralMode(NeutralMode.Brake);
    field = new Field2d();
    SmartDashboard.putData(field);
    SmartDashboard.putData(this);
    for (SwerveModule m : modules) {
      SmartDashboard.putData(m.name, m);
    }
    modules[0].debug = true;

    SmartDashboard.putNumber("Set Gyro Angle", 0);
    SmartDashboard.putData("Change gyro angle ", new InstantCommand( () -> setGyroAngle(SmartDashboard.getNumber("Set Gyro Angle", 0))).ignoringDisable(true));


    SmartDashboard.putData("run command", new RunCommand(()->{setModulesPower(1); setModulesAngleFromSB(0);}, this));

    SmartDashboard.putData("set coast",
        new InstantCommand(() -> setNeutralMode(NeutralMode.Coast)).ignoringDisable(true));
    SmartDashboard.putData("set brake",
        new InstantCommand(() -> setNeutralMode(NeutralMode.Brake)).ignoringDisable(true));
    SmartDashboard.putData("reset wheels", new InstantCommand(() -> resetWheels()).ignoringDisable(true));
        SmartDashboard.putData("reset pose", new InstantCommand(() -> setOdometryToForward()).ignoringDisable(true));

    SmartDashboard.putData("calibrate", new InstantCommand(()->calibrate(), this).ignoringDisable(true));
    
    pathPoint dummyPoint = new pathPoint(0, 0, new Rotation2d(), 0, false);
    pathPoint point = new pathPoint(2, 1, Rotation2d.fromDegrees(-20), 0, false);

    Command cmdpf = new PathFollow(this, new pathPoint[] { dummyPoint, point }, 4.1, 10, 0, false);
   
    SmartDashboard.putData("check offset spin", new DriveStraightLine(this));
    SmartDashboard.putData("Path Follow Check", cmdpf);
    SmartDashboard.putData("Chassis Move Sysid",
        (new Sysid(this::setModulesPower, this::getMoveVelocity, 0.2, 0.8, this)).getCommand());
    SmartDashboard.putData("Chassis Move Sysid2",
        (new Sysid(new Gains[] { Gains.KS, Gains.KV, Gains.KA, Gains.KV2, Gains.KVsqrt},
        this::setModulesPower,
        this::getMoveVelocity,
        null,
        null,
        0.1,
        0.6,
        3,
        1,
        1,
        this)).getCommand());
   // SmartDashboard.putData("Test Steer Velocity", (new CheckModulesSteerVelocity(this, 200)));
   // SmartDashboard.putData("Set Modules Angle", (new SetModuleAngle(this)));
   // new TestVelocity("Chassis", this::setVelocity, this::getMoveVelocity, 0.05, this);
   // SmartDashboard.putNumber("angle for chassis", 0);
   // SmartDashboard.putData("go to 0", new RunCommand(()->setModulesAngleFromSB(SmartDashboard.getNumber("angle for chassis", 0)), this));

   // SmartDashboard.putNumber("ANG", 0);
   // SmartDashboard.putData("go to angle position", new RunCommand(()->modules[0].setAngleByPositionPID(Rotation2d.fromDegrees(SmartDashboard.getNumber("ANG", 0))), this));

   setNeutralMode(NeutralMode.Brake);
   SmartDashboard.putData("SetAngle45", new RunCommand(() -> {
    for (SwerveModule module : modules) {
      module.setAngleByPositionPID(Rotation2d.fromDegrees(45));  
    }
   }, this));
  }

  public static double getTargetVelocity(){
    return targetVelocity;
  }
  public static double getVelocityAsDouble(){
    return currentVelocity;
  }

  public SwerveModule[] getModules() {
    return modules;
  }

  public void calibrate() {
    SmartDashboard.putNumber("LEFT FRONT", modules[0].getAngleDegreesRaw());
    SmartDashboard.putNumber("RIGHT FRONT", modules[1].getAngleDegreesRaw());
    SmartDashboard.putNumber("LEFT BACK", modules[2].getAngleDegreesRaw());
    SmartDashboard.putNumber("RIGHT BACK", modules[3].getAngleDegreesRaw());
  }

  public boolean isRed() {
    return RobotContainer.robotContainer.isRed();
  }

  public void setGyroAngle(double angle){
    gyro.setYaw(angle);
  }

  public SwerveModule getModule(int i) {
    return modules[i];
  }

  public void resetWheels() {
    for (var module : modules) {
      module.setAngleByPositionPID(new Rotation2d());
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

  public void setModulesPower(double power) {
    for (var m : modules) {
      m.setPower(power);
    }
  }

  public void setModulesSteerVelocity(double v) {
    for (var m : modules) {
      m.setSteerVelocity(v, false);
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
    double[] angularVelocities = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      angularVelocities[i] = modules[i].getVelocity();
    }
    return angularVelocities;
  }

  public void setPose(Pose2d pose){
    poseEstimator.resetPosition(getRawAngle(), getModulePositions(), pose);
  }

  /**
   * Sets the velocity of the chassis
   * 
   * @param speeds In m/s and rad/s
   */
  public void setVelocities(ChassisSpeeds speeds) {
    double param = speeds.omegaRadiansPerSecond > Math.toRadians(20) ? -0.1 : 0;
    ChassisSpeeds relativeSpeeds = 
    ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
    Translation2d newSpeeds = new Translation2d(relativeSpeeds.vxMetersPerSecond,
     relativeSpeeds.vyMetersPerSecond).rotateBy(Rotation2d.fromRadians(relativeSpeeds.omegaRadiansPerSecond * param));
    ChassisSpeeds newChassisSpeeds = new ChassisSpeeds(newSpeeds.getX(), newSpeeds.getY(), relativeSpeeds.omegaRadiansPerSecond);
    newChassisSpeeds.omegaRadiansPerSecond = SwerveKinematics.fixOmega(newChassisSpeeds.omegaRadiansPerSecond);
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(newChassisSpeeds);
    targetVelocity = new Translation2d(newChassisSpeeds.vxMetersPerSecond, newChassisSpeeds.vyMetersPerSecond).getNorm();
    setModuleStates(states);
  }

  public void setVelocitiesRobotRel(ChassisSpeeds speeds){
    
    setModuleStates(KINEMATICS.toSwerveModuleStates(speeds));
  }



// public void setVelocities(ChassisSpeeds speeds) {
//   ChassisSpeeds s = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
//   SwerveModuleState[] states = KINEMATICS_CORRECTED.toSwerveModuleStates(s);
//   setModuleStates(states);
// }

  public void setVelocitiesRotateToSpeaker(ChassisSpeeds speeds) {
    speeds.omegaRadiansPerSecond = getRadPerSecToSpeaker();
    setVelocities(speeds);
  }
  public void setVelocitiesRotateToAngle(ChassisSpeeds speeds, Rotation2d angle) {
    speeds.omegaRadiansPerSecond = getRadPerSecToAngle(angle);
    setVelocities(speeds);
  }


  public void configAllFactoryDefaut() {
    for (int i=0; i<modules.length; i++) {
      modules[i].configFactoryDefault();
    }
  }

  public double getErrorSpeakerAngle(){
    return speakerAngleError;
  }




  /**
   * Returns the velocity vector
   * 
   * @return Velocity in m/s
   */
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
  public void setNeutralMode(NeutralMode mode) {
    Arrays.stream(modules).forEach((module) -> module.setNeutralMode(mode));
  }


  /**
   * Returns the angle of the gyro
   */
  public Rotation2d getRawAngle() {
    return Rotation2d.fromDegrees(gyro.getYaw());
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

  public double[] getModulesAngles() {
    double[] angles = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      angles[i] = modules[i].getAngleDegrees();
    }
    return angles;
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

  public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator(){
    return this.poseEstimator;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("velocity", () -> getVelocity().getNorm(), null);
    builder.addDoubleProperty("Omega", () -> Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond), null);
    builder.addDoubleProperty("Angle", () -> Utils.degrees(getAngle()), null);
    builder.addDoubleProperty("Pitch", () -> gyro.getPitch(), null);
    builder.addDoubleProperty("Roll", () -> gyro.getRoll(), null);
    builder.addDoubleProperty("Pose X", this::getPoseX, null);
    builder.addDoubleProperty("Pose Y", this::getPoseY, null);
    SmartDashboard.putData("Set Modules Angle", new RunCommand(() -> setModulesAngleFromSB(0)));
  }

  public void setModulesAngleFromSB(double angle) {
    Rotation2d a = Rotation2d.fromDegrees(angle);
    for (SwerveModule module : modules) {
      module.setAngleByPositionPID(a);
    }
  }

  
  public Rotation2d getClosetAngleApriltag(){
    Translation2d finalVector = new Translation2d(Integer.MAX_VALUE, Integer.MAX_VALUE);
    //checks the distance from each april tag and finds
    for(int i = 0; i < aprilTagsPositions.length; i++){
      Translation2d currentAprilTagVector = getPose().minus(aprilTagsPositions[i]).getTranslation();

     if(currentAprilTagVector.getNorm() < finalVector.getNorm()){
      finalVector = currentAprilTagVector;
     }
    }
    return finalVector.getAngle();
  }

  public void GoToAMP(){
    double kP = 0.2;
    Translation2d AMP = LimelightVisionUtils.getDxDyAMP();
    if(AMP != null){
      setVelocitiesRotateToAngle(new ChassisSpeeds(0, -AMP.getY() * kP,
      Math.toRadians(180)), getAngle().minus(LimelightVisionUtils.getTAAmp()));
    }
    else {
      setVelocities(new ChassisSpeeds(0, 0, 0));
    }
  }

  public double getRadPerSecToSpeaker() {
    Translation2d speaker = Utils.speakerTargetPosition();
    Rotation2d limelightSpeakerAngle = RobotContainer.robotContainer.vision.getSpeakerAngle();
    Rotation2d odometryAngle = speaker.minus(getPose().getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180));
    double dif = (limelightSpeakerAngle!=null && odometryAngle!=null) ? Math.abs(Utils.angelErrorInDegrees(limelightSpeakerAngle, odometryAngle, 10)) : 1000;
    Rotation2d target = limelightSpeakerAngle != null && distanceFromSpeaker() < 3 && dif == 0 ?
      limelightSpeakerAngle : odometryAngle;
    return getRadPerSecToAngle(target);
  }

  double lastAngle = 0;
  double currentAngle = 0;
  public double getGyroRate() {
    return (currentAngle - lastAngle)/0.02;
  }

  public double getGyroRateZ() {
    double []rot  = new double[3];
    gyro.getRawGyro(rot);
    return rot[2];
  }

  public double getGyroRateY() {
    double []rot  = new double[3];
    gyro.getRawGyro(rot);
    return rot[1];
  }

  public double getGyroRateX() {
    double []rot  = new double[3];
    gyro.getRawGyro(rot);
    return rot[0];
  }

  public double getRadPerSecToAngle(Rotation2d fieldRelativeAngle) {
      speakerAngleError = Utils.angelErrorInDegrees(fieldRelativeAngle, getAngle(),4);
      double rotateVel = angleSpeakerPID.calculate(-speakerAngleError,0);

      return rotateVel;
  }
  
  public static Translation2d speakerPosition() {
    return Utils.speakerPosition();
  }

  public boolean isAimingSpeaker() {
    return isAimingSpeaker;
  }


  public double distanceFromSpeaker() {
    return speakerPosition().getDistance(getPose().getTranslation());
  }

  @Override
  public void periodic() {
    poseEstimator.update(getRawAngle(), getModulePositions());
    lastAngle = currentAngle;
    currentAngle = getRawAngle().getDegrees();

    currentVelocity = getVelocity().getNorm();

    field.setRobotPose(getPose().plus(new Transform2d(0, 0, new Rotation2d())));
    SmartDashboard.putNumber("Distance from speaker", speakerPosition().getDistance(getPose().getTranslation()));
 }
}