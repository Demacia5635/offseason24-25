// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.*;

public class SysIdCmd extends Command {
  /** Creates a new SysIdCmd. */
  private TalonFX motor;
  private double power;
  private String nameForShuffleBoard;

  public SysIdCmd(TalonFX motor, double power, String nameForShuffleBoard) {
    this.motor = motor;
    this.power = power;
    this.nameForShuffleBoard = nameForShuffleBoard;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motor.set(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber(nameForShuffleBoard, motor.getVelocity().getValue()/GEAR_RATIO*Scope);
    motor.set(0);
  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
