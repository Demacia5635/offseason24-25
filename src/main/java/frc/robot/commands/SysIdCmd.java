// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SysIdDefault;
import frc.robot.subsystems.TestSubsystem;

import static frc.robot.Constants.*;

public class SysIdCmd extends Command {
  /** Creates a new SysIdCmd. */
  private TalonFX motor;
  private double power;
  private String nameForShuffleBoard;
  private TestSubsystem subsystem = new TestSubsystem();
  private SysIdDefault sysId = new SysIdDefault();

  public SysIdCmd(double power, String nameForShuffleBoard) {
    //this.motor = motor;
    this.power = power;
    this.nameForShuffleBoard = nameForShuffleBoard;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setPowers(power);
    sysId.showFeedForward(-0.1,0.2,SCOPE);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber(nameForShuffleBoard, subsystem.getTrueVelocity());
    //Kv = sysId.getKV(0.1,0.2,SmartDashboard.getNumber("v1", 0),SmartDashboard.getNumber("v2", 0));
    //Ks = sysId.getKS(0.1, Kv, SmartDashboard.getNumber("v1", 0));

    System.out.println(SmartDashboard.getNumber("ks", 0));
    System.out.println(SmartDashboard.getNumber("kv", 0));
    System.out.println(SmartDashboard.getNumber("KV", 0));
    System.out.println(SmartDashboard.getNumber("KS", 0));
    subsystem.setPower(0);
  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
