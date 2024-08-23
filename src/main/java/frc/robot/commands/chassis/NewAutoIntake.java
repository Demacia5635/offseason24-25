// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.Constants.ChassisConstants;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_ACCELERATION;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_VELOCITY;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_DRIVE_VELOCITY;
import static frc.robot.subsystems.chassis.ChassisConstants.METER_IN_CM;

public class NewAutoIntake extends Command {
  Chassis chassis;
  double velocity;
  double maxVelocity;
  double lastDistance;
  double angle;
  double fieldRelativeAngle;

  double DISTANCE_OFFSET = 0.1; //in meters
  double SPEED_OFFSET_WHEN_MAX = -1;

  double[] llpython;
  double distance;
  NetworkTableEntry llentry;
  long lastCounter;
  Translation2d notePos;
  CommandXboxController controller;
  PIDController pid = new PIDController(1, 0, 0);
  public NewAutoIntake(Chassis chassis, CommandXboxController controller) {
    this.chassis = chassis;
    this.controller = controller;
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
      fieldRelativeAngle = angle + chassis.getAngle().getDegrees();

      ChassisSpeeds speeds;
    
      Translation2d vectorToNote = new Translation2d(calcVectorLength(), fieldRelativeAngle);
      Translation2d alignmentVector = calcAlignmentVector();
      Translation2d velocityVector = vectorToNote.plus(alignmentVector);
      speeds = new ChassisSpeeds(velocityVector.getX(), velocityVector.getY(), 0);

      chassis.setVelocitiesRotateToAngle(speeds, Rotation2d.fromDegrees(fieldRelativeAngle));
      
      



    }
  }

  private Translation2d calcAlignmentVector(){
    double vectorLength = pid.calculate(angle, 0);
    double vectorAngle = fieldRelativeAngle + (Math.signum(angle) * 90);
    return new Translation2d(vectorLength, Rotation2d.fromDegrees(vectorAngle));
    
  }
  private Translation2d getStickVector(){
    return new Translation2d(controller.getLeftX(), controller.getLeftY());
  } 
  private double calcVectorLength(){
    
    double angle = fieldRelativeAngle;
    double highBound = angle + 90;
    double lowBound = angle - 90;
    return getStickVector().getNorm() * MAX_DRIVE_VELOCITY *
    angle > lowBound && angle < highBound ? 1 : -1;
  }


  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
