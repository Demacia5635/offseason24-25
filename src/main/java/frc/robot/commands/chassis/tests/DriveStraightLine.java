// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis.tests;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.chassis.Chassis;

public class DriveStraightLine extends Command {
  Chassis chassis;
  double vel;
  public DriveStraightLine(Chassis chassis) {
    this.chassis = chassis;
    
    SmartDashboard.putNumber("Vel check spin", 0);
    addRequirements(chassis);
    SmartDashboard.putData(this);
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);  }

  
  @Override
  public void initialize() {
    chassis.setPose(new Pose2d());
  }

 
  @Override
  public void execute() {
    vel = 3;
    System.out.println("Vel check spin" + vel);
    chassis.setVelocities(new ChassisSpeeds(vel, 0, 3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(chassis.getPoseX()) >= 3;
  }
}
