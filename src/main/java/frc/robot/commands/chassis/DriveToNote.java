package frc.robot.commands.chassis;

import static frc.robot.subsystems.chassis.ChassisConstants.COLLECT_OFFSET_METERS;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_DRIVE_VELOCITY;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_ACCELERATION;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_VELOCITY;
import static frc.robot.subsystems.chassis.ChassisConstants.METER_IN_CM;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import edu.wpi.first.wpilibj.Timer;

public class DriveToNote extends Command {
  Chassis chassis;
  double velocity;
  double maxVelocity;
  double lastDistance;

  double[] llpython;
  double distance;
  double angle;
  double lastAngle;
  NetworkTableEntry llentry;
  ChassisSpeeds speed;
  boolean finish = false;
  boolean countTime;
  long lastCounter;
  double fieldRelativeAngle;

  PIDController rotationPidController = new PIDController(0.04, 0.00, 0.006);

  Timer timer = new Timer();

  public static boolean isStart = false;

  public DriveToNote(Chassis chassis, double vel, boolean countTime) {
    this.chassis = chassis;
    this.maxVelocity = vel;
    this.countTime = countTime;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    isStart = true;
    lastDistance = 0;
    distance = 0;
    llentry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython");
    llpython = llentry.getDoubleArray(new double[8]);
    finish = llpython[0] == 0;
    timer.start();
  }

  private double calcTimeToRotate(double angle) {
    angle = Math.toRadians(angle);
    return 2 * (Math.sqrt(Math.abs(angle) / MAX_OMEGA_ACCELERATION));
  }

  @Override
  public void execute() {
    if (finish) {
      return;
    }
    llpython = llentry.getDoubleArray(new double[8]);
    if (llpython[2] != lastCounter) {
      lastCounter = (long)llpython[2];
      distance = llpython[0];
      angle = llpython[1] - chassis.getGyroRate() * 0.05;
      //System.out.println("note distance= " + distance + ", note angle= " + angle);
      if (distance > 0) {
        timer.reset();
      } else {
        return;
      }
      // double rotateVel = (Math.abs(angle - 3) <= 3) ? 0 :
      // rotationPidController.calculate(-angle, 3);
      fieldRelativeAngle = angle + chassis.getAngle().getDegrees();
      double distanceMeters = distance / METER_IN_CM;

      fieldRelativeAngle = Math.toRadians(fieldRelativeAngle);
      double time = calcTimeToRotate(angle);
      if (time < 0.1) {
        velocity = maxVelocity;
      } else {
        velocity = Math.min((distanceMeters - COLLECT_OFFSET_METERS) / time, maxVelocity);
      }
     // System.out.println("drive velocity= " + velocity);
      speed = new ChassisSpeeds(velocity * Math.cos(fieldRelativeAngle), velocity * Math.sin(fieldRelativeAngle),
          MAX_OMEGA_VELOCITY);
      lastDistance = distance;
      lastAngle = fieldRelativeAngle;
    }
    chassis.setVelocitiesRotateToAngle(speed, Rotation2d.fromRadians(fieldRelativeAngle));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
    isStart = false;
  }

  @Override
  public boolean isFinished() {
    return finish || (timer.get() > -(0.5 * velocity) + 1.25 && countTime);
  }

}