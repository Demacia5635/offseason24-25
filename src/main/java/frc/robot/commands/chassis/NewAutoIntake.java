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

  double DISTANCE_OFFSET = 0.1; //in meters

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




  @Override
  public void execute() {
    llpython = llentry.getDoubleArray(new double[8]);
    if (llpython[2] != lastCounter) {
      lastCounter = (long)llpython[2];
      distance = llpython[0];
      angle = llpython[1] - chassis.getGyroRate() * 0.05;

      notePos = chassis.getPose().getTranslation().plus(new Translation2d(distance, angle + chassis.getAngle().getDegrees()));
      double verticalDistance = notePos.getX() - chassis.getPoseX();
      double horizontalDistance = notePos.getY() - chassis.getPoseY();
      double vY = getTimeToNote(verticalDistance) * horizontalDistance;
      chassis.setVelocities(new ChassisSpeeds(chassis.getChassisSpeeds().vxMetersPerSecond, vY, 0));



    }
  }

  private double getTimeToNote(double distance){
    return (distance - 0.1) / chassis.getVelocity().getNorm();
  }

  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
