package frc.robot.subsystems.chassis;


import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Sysid.Sysid;
import frc.robot.Sysid.Sysid.Gains;
import frc.robot.subsystems.chassis.utils.SwerveKinematics;

import frc.robot.utils.Utils;
import static frc.robot.subsystems.chassis.ChassisConstants.*;

import java.util.Arrays;

import com.ctre.phoenix6.hardware.Pigeon2;



public class Chassis extends SubsystemBase {
  private final SwerveModule[] modules;
  private final Pigeon2 gyro;



  private SwerveDrivePoseEstimator poseEstimator;


  private final Field2d field;



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
    for (SwerveModule m : modules) {
      SmartDashboard.putData(m.name, m);
    }
    modules[0].debug = true;

    SmartDashboard.putNumber("Set Gyro Angle", 0);
    SmartDashboard.putData("Change gyro angle ", new InstantCommand( () -> setGyroAngle(SmartDashboard.getNumber("Set Gyro Angle", 0))).ignoringDisable(true));


    SmartDashboard.putData("run command", new RunCommand(()->{setModulesPower(1); setModulesAngleFromSB(0);}, this));

    SmartDashboard.putData("set coast",
        new InstantCommand(() -> setBrake(false)).ignoringDisable(true));
    SmartDashboard.putData("set brake",
        new InstantCommand(() -> setBrake(true)).ignoringDisable(true));
    SmartDashboard.putData("reset wheels", new InstantCommand(() -> resetWheels()).ignoringDisable(true));
        SmartDashboard.putData("reset pose", new InstantCommand(() -> setOdometryToForward()).ignoringDisable(true));

    SmartDashboard.putData("calibrate", new InstantCommand(()->calibrate(), this).ignoringDisable(true));


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

   setBrake(true);
   SmartDashboard.putData("SetAngle45", new RunCommand(() -> {
    for (SwerveModule module : modules) {
      module.setAngleByPositionPID(Rotation2d.fromDegrees(45));  
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
    setModuleStates(states);
  }


// public void setVelocities(ChassisSpeeds speeds) {
//   ChassisSpeeds s = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
//   SwerveModuleState[] states = KINEMATICS_CORRECTED.toSwerveModuleStates(s);
//   setModuleStates(states);
// }












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
  public void setBrake(boolean brake) {
    Arrays.stream(modules).forEach((module) -> module.setBrake(brake));
  }


  /**
   * Returns the angle of the gyro
   */
  public Rotation2d getRawAngle() {
    return Rotation2d.fromDegrees(gyro.getAngle());
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

  



  

  double lastAngle = 0;
  double currentAngle = 0;
  public double getGyroRate() {
    return (currentAngle - lastAngle)/0.02;
  }



  
 

  @Override
  public void periodic() {
    poseEstimator.update(getRawAngle(), getModulePositions());
    lastAngle = currentAngle;
    currentAngle = getRawAngle().getDegrees();

    field.setRobotPose(getPose().plus(new Transform2d(0, 0, new Rotation2d())));
 }
}