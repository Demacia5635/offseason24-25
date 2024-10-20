package frc.robot.chassis.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.chassis.subsystems.Chassis;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.chassis.ChassisConstants.*;

public class DriveToNote extends Command {
  Chassis chassis;
  double velocity;
  double maxVelocity;
  double lastDistance;
  double angle;
  double lastAngle;
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
    finish = false;
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
  
    angle = chassis.visionByNote.getAngleToNote().getDegrees();
    fieldRelativeAngle = angle + chassis.getAngle().getDegrees();
    

    fieldRelativeAngle = Math.toRadians(fieldRelativeAngle);
    double time = calcTimeToRotate(angle);
    if (time < 0.1) {
      velocity = maxVelocity / 2;
    } else {
      velocity = maxVelocity;
    }
     // System.out.println("drive velocity= " + velocity);
    speed = new ChassisSpeeds(velocity * Math.cos(fieldRelativeAngle),
      velocity * Math.sin(fieldRelativeAngle),
      MAX_OMEGA_VELOCITY);
    lastAngle = fieldRelativeAngle;
    
    chassis.setVelocitiesRotateToAngle(speed, Rotation2d.fromRadians(fieldRelativeAngle));

    if(RobotContainer.robotContainer.intake.isNoteInIntake) finish = true;
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