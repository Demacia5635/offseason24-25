// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.Constants.ChassisConstants;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_ACCELERATION;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_VELOCITY;
import static frc.robot.subsystems.chassis.ChassisConstants.METER_IN_CM;

public class NewAutoIntake extends Command {
  Chassis chassis;
  double velocity;
  double maxVelocity;
  double lastDistance;
  double angle;
  double fieldRelativeAngle;

  double[] llpython;
  double distance;
  NetworkTableEntry llentry;
  long lastCounter;
  Translation2d notePos;
  public NewAutoIntake(Chassis chassis) {
    this.chassis = chassis;
  }

  
  @Override
  public void initialize() {
    llentry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython");
    llpython = llentry.getDoubleArray(new double[8]);
    distance = 0;
  }

  private double calcTimeToRotate(double angle) {
    angle = Math.toRadians(angle);
    return 2 * (Math.sqrt(Math.abs(angle) / MAX_OMEGA_VELOCITY));
  }



  @Override
  public void execute() {
    llpython = llentry.getDoubleArray(new double[8]);
    if (llpython[2] != lastCounter) {
      lastCounter = (long)llpython[2];
      distance = llpython[0];
      angle = llpython[1] - chassis.getGyroRate() * 0.05;
      fieldRelativeAngle = angle + chassis.getAngle().getDegrees();

      distance = distance / METER_IN_CM;
      notePos = chassis.getPose().getTranslation().plus(new Translation2d(distance, fieldRelativeAngle));
      double curVx = chassis.getChassisSpeeds().vxMetersPerSecond;
      double yLength = notePos.getY() - chassis.getPoseY();
      ChassisSpeeds speeds = new ChassisSpeeds(curVx, yLength + 0.02 , MAX_OMEGA_VELOCITY);
      chassis.setVelocitiesRotateToAngle(speeds, null);



    }
  }

  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
