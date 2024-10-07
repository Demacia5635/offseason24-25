package frc.robot.chassis.subsystems;


import java.util.Arrays;

import com.ctre.phoenix6.hardware.Pigeon2;

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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Sysid.Sysid;
import frc.robot.Sysid.Sysid.Gains;
import static frc.robot.chassis.ChassisConstants.BACK_LEFT;
import static frc.robot.chassis.ChassisConstants.BACK_RIGHT;
import static frc.robot.chassis.ChassisConstants.FRONT_LEFT;
import static frc.robot.chassis.ChassisConstants.FRONT_RIGHT;
import static frc.robot.chassis.ChassisConstants.GYRO_ID;
import static frc.robot.chassis.ChassisConstants.KINEMATICS;
import static frc.robot.chassis.ChassisConstants.MAX_DRIVE_VELOCITY;

import frc.robot.utils.LogManager;



public class Chassis extends SubsystemBase {
  private final SwerveModule[] modules;
  private final Pigeon2 gyro;
  private SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field;

  private PIDController rotationPID = new PIDController(0.05,0.0, 0.002);

  public Chassis() {
    modules = new SwerveModule[] {
        new SwerveModule(FRONT_LEFT, this),
        new SwerveModule(FRONT_RIGHT, this),
        new SwerveModule(BACK_LEFT, this),
        new SwerveModule(BACK_RIGHT, this),
    };    
    gyro = new Pigeon2(GYRO_ID);
    gyro.setYaw(0);
    
    poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getRawAngle(), getModulePositions(), new Pose2d());
    
    setBrake(true);
    field = new Field2d();
    SmartDashboard.putData(field);
    SmartDashboard.putData(this);
    modules[0].debug = true;

    ShuffleboardTab ntTab = Shuffleboard.getTab("Chassis");
    ntTab.add("Chassis", this);
    //var setGyroEntry = ntTab.add("Set Gyro Angle", 0).getEntry();
    ntTab.add("Set gyro to 0 ",
       new InstantCommand( () -> setGyroAngle(0))
       .ignoringDisable(true));


    ntTab.add("run command", 
      new RunCommand(()->{setModulesPower(1); setModulesAngleFromSB(0);}, this));

    ntTab.add("set coast",
        new InstantCommand(() -> setBrake(false)).ignoringDisable(true));
    ntTab.add("set brake",
        new InstantCommand(() -> setBrake(true)).ignoringDisable(true));
    ntTab.add("reset wheels", new InstantCommand(() -> resetWheels()).ignoringDisable(true));
        SmartDashboard.putData("reset pose", new InstantCommand(() -> setOdometryToForward()).ignoringDisable(true));

    ntTab.add("calibrate", new InstantCommand(()->calibrate(), this).ignoringDisable(true));


    ntTab.add("Chassis Move Sysid",
        (new Sysid(this::setModulesPower, this::getMoveVelocity, 0.2, 0.8, this)).getCommand());
    ntTab.add("Chassis Move Sysid2",
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
    ntTab.add("Set Modules Angle", new RunCommand(() -> setModulesAngleFromSB(0)).ignoringDisable(true));
   // SmartDashboard.putData("Test Steer Velocity", (new CheckModulesSteerVelocity(this, 200)));
   // SmartDashboard.putData("Set Modules Angle", (new SetModuleAngle(this)));
   // new TestVelocity("Chassis", this::setVelocity, this::getMoveVelocity, 0.05, this);
   // SmartDashboard.putNumber("angle for chassis", 0);
   // SmartDashboard.putData("go to 0", new RunCommand(()->setModulesAngleFromSB(SmartDashboard.getNumber("angle for chassis", 0)), this));

   // SmartDashboard.putNumber("ANG", 0);
   // SmartDashboard.putData("go to angle position", new RunCommand(()->modules[0].setAngleByPositionPID(Rotation2d.fromDegrees(SmartDashboard.getNumber("ANG", 0))), this));

   setBrake(true);
   ntTab.add("SetAngle45", new RunCommand(() -> {
    Rotation2d angle = Rotation2d.fromDegrees(45);
    for (SwerveModule module : modules) {
      module.setSteerPosition(angle);  
    }
   }, this));
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
    Rotation2d angle = Rotation2d.fromDegrees(0);
    for (var module : modules) {
      module.setSteerPosition(angle);
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

  public void speen(double speed){
    setVelocities(new ChassisSpeeds(0,0,speed));
  }

  /**
   * Sets the velocity of the chassis
   * 
   * @param speeds In m/s and rad/s
   */
  public void setVelocities(ChassisSpeeds speeds) {
    ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(relativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_DRIVE_VELOCITY);

    // System.out.println("o" + speeds.omegaRadiansPerSecond);
    // System.out.println("vX" + speeds.vxMetersPerSecond);
    // System.out.println("vY" + speeds.vyMetersPerSecond);
    setModuleStates(states);
  }

  // public void setVelocities(ChassisSpeeds speeds){
  //   SwerveModuleState[] states = KINEMATICS_DEMACIA.toSwerveModuleStates(speeds, getPose(), getModuleStates());
  //   SwerveKinematics.desaturateWheelSpeeds(states, MAX_DRIVE_VELOCITY);
  //   setModuleStates(states);
  // }

  public void setVelocitiesRotateToAngle(ChassisSpeeds speeds, Rotation2d angle) {
    speeds.omegaRadiansPerSecond = getRadPerSecToAngle(angle);
    setVelocities(speeds);
  }

  public double getRadPerSecToAngle(Rotation2d fieldRelativeAngle){
    return rotationPID.calculate(-fieldRelativeAngle.getRadians(), 0);
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

  public void setModulesSteerPosition(Rotation2d angle, int i) {
    modules[i].setSteerPosition(angle);

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
      module.setSteerPosition(a);
    }
  }


  public double getGyroRate() {
    return gyro.getRate();
  }

  @Override
  public void periodic() {
    poseEstimator.update(getRawAngle(), getModulePositions());
    field.setRobotPose(getPose().plus(new Transform2d(0, 0, new Rotation2d())));
 }
}