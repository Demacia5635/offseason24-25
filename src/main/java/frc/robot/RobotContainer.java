
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.LogManager;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.commands.DriveCommand;
import frc.robot.chassis.subsystems.Chassis;

import frc.robot.Shooter.Commands.GoToAngle;
import frc.robot.Shooter.Commands.Shoot;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.Subsystems.Shooter;

public class RobotContainer implements Sendable{
  public static RobotContainer robotContainer;
  private Boolean isRed = true;
  CommandXboxController controller;

  public LogManager logManager = new LogManager();

  public Chassis chassis;
  public static Shooter shooter;
  public static AngleChanger angleChanging;
  private Shoot shootCommand;

  public static boolean isShooterReady = false;

  public Command resetOdometry;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    robotContainer = this;
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    chassis = new Chassis();
    controller = new CommandXboxController(0);
    chassis.setDefaultCommand(new DriveCommand(chassis, controller));
    shooter = new Shooter();
    angleChanging = new AngleChanger();
    angleChanging.setDefaultCommand(new GoToAngle(angleChanging));
    shootCommand = new Shoot(shooter);

    SmartDashboard.putData("RobotContainer", this);

    // Configure the trigger bindings
    createCommands();
    configureBindings();
  }

  private void createCommands() {
    resetOdometry = new InstantCommand(()-> chassis.setOdometryToForward()).ignoringDisable(true);
  }


  private void configureBindings() {

    controller.back().onTrue(resetOdometry);
    
      controller.a().onTrue(shootCommand);

    controller.b().onTrue(new InstantCommand(() -> {
      if (shooter.shooterState == STATE.SPEAKER){
        shooter.shooterState = STATE.AMP;
        angleChanging.angleState = STATE.AMP;
      }
      if (shooter.shooterState == STATE.AMP){
        shooter.shooterState = STATE.SPEAKER;
        angleChanging.angleState = STATE.SPEAKER;
      }
    }, shooter, angleChanging));

    controller.x().onTrue(new InstantCommand(() -> {
        isShooterReady = true;
    }, shooter));

    controller.y().onTrue(new InstantCommand(() -> {
        shooter.shooterState = STATE.TESTING;
        angleChanging.angleState = STATE.TESTING;
    }, shooter).ignoringDisable(true));

    controller.rightTrigger().onTrue(new InstantCommand(() -> {
        shooter.shooterState = STATE.STAGE;
        angleChanging.angleState = STATE.STAGE;
    }, shooter, angleChanging));

    controller.leftTrigger().onTrue(new InstantCommand(() -> {
        shooter.shooterState = STATE.SUBWOFFER;
        angleChanging.angleState = STATE.SUBWOFFER;
    }, shooter, angleChanging));

    controller.back().onTrue(new InstantCommand(() -> {
        shooter.shooterState = STATE.IDLE;
        angleChanging.angleState = STATE.IDLE;
    }, shooter, angleChanging));
  }
   
  public void isRed(boolean isRed) {
    this.isRed = isRed;
  }
  public boolean isRed() {
    return isRed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is Red",this::isRed, this::isRed);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}