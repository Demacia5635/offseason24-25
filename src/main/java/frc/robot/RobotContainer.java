
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;


import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.vision.subsystem.visionByNote;
import frc.robot.vision.subsystem.visionByTag;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Intake.Command.IntakeCommand;
import frc.robot.Intake.Subsystem.Intake;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Commands.Calibration;
import frc.robot.Shooter.Commands.GoToAngle;
import frc.robot.Shooter.Commands.Shoot;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.Subsystems.Shooter;
import frc.robot.chassis.commands.DriveCommand;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;

import static frc.robot.Constants.*;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer implements Sendable{
  public static RobotContainer robotContainer;
  public static Boolean isRed = true;
  CommandXboxController controller;

  public LogManager logManager = new LogManager();

  public static Chassis chassis;
  public static Shooter shooter;
  public static AngleChanger angleChanging;
  public static Intake intake;

  private IntakeCommand intakeCommand;
  private Shoot shootCommand;
  private Command resetOdometry;
  private DriveCommand driveCommand;
  private GoToAngle gotoAngleCommand;
  private Calibration calibration;

  public static boolean isDriverOverwriteShooter = false;

  Pigeon2 gyro;

  visionByTag pose;
  visionByNote note;
  Field2d field;

  public RobotContainer() {

    robotContainer = this;
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    controller = new CommandXboxController(CONTROLLER_PORT);

    chassis = new Chassis();
    shooter = new Shooter();
    angleChanging = new AngleChanger();
    intake = new Intake();

    shootCommand = new Shoot(shooter, intake, chassis);
    intakeCommand = new IntakeCommand(intake);
    resetOdometry = new InstantCommand(()-> chassis.setOdometryToForward())
                        .ignoringDisable(true);
    driveCommand = new DriveCommand(chassis, controller);
    gotoAngleCommand = new GoToAngle(angleChanging, chassis);
    
    chassis.setDefaultCommand(driveCommand);
    angleChanging.setDefaultCommand(gotoAngleCommand);
    
    gyro = chassis.gyro;
    pose = new visionByTag(gyro);
    note = new visionByNote(pose.getRoobotPose());

    SmartDashboard.putData("RobotContainer", this);
    SmartDashboard.putData("fiset gyro", new InstantCommand(()->gyro.setYaw(0)));

    configureBindings();
  }


  private void configureBindings() {

    controller.back().onTrue(resetOdometry);
    
    controller.y().onTrue(new RunCommand(()-> {
      intake.setPowerToMotors(-1);
    }, intake));
    controller.a().onTrue(intakeCommand);
    
    controller.rightBumper().onTrue(new InstantCommand(()->isDriverOverwriteShooter = true));
    controller.x().onTrue(shootCommand);
    
    controller.start().onTrue(calibration);
    controller.povRight().onTrue(new InstantCommand(()-> {
      if (shooter.shooterState == STATE.AMP
          || shooter.shooterState == STATE.IDLE) {
        shooter.shooterState = STATE.SPEAKER;

      } else if (shooter.shooterState == STATE.SPEAKER 
                || shooter.shooterState == STATE.SUBWOFFER 
                || shooter.shooterState == STATE.STAGE 
                || shooter.shooterState == STATE.DELIVERY_MID 
                || shooter.shooterState == STATE.DELIVERY_RIVAL) {
        shooter.shooterState = STATE.AMP;
      }

      if (angleChanging.angleState == STATE.AMP
          || angleChanging.angleState == STATE.IDLE) {
        angleChanging.angleState = STATE.SPEAKER;

      } else if (angleChanging.angleState == STATE.SPEAKER 
                || angleChanging.angleState == STATE.SUBWOFFER 
                || angleChanging.angleState == STATE.STAGE 
                || angleChanging.angleState == STATE.DELIVERY_MID 
                || angleChanging.angleState == STATE.DELIVERY_RIVAL) {
        angleChanging.angleState = STATE.AMP;
      }
    }));
    controller.povLeft().onTrue(new InstantCommand(()-> {
      shooter.shooterState = STATE.TESTING;
      angleChanging.angleState = STATE.TESTING;
    }));
    controller.povDown().onTrue(new InstantCommand(()-> {
      shooter.shooterState = STATE.SUBWOFFER;
      angleChanging.angleState = STATE.SUBWOFFER;
    }));
    controller.povUp().onTrue(new InstantCommand(()-> {
      shooter.shooterState = STATE.SPEAKER;
      angleChanging.angleState = STATE.SPEAKER;
    }));

    controller.leftBumper().onTrue(stopAll());
  }
   
  public void isRed(boolean isRed) {
    RobotContainer.isRed = isRed;
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

  public Command calibration() {
    return calibration;
  }

  public Command stopAll() {
    return new InstantCommand(()-> {
      chassis.stop();
      shooter.setMotorPower(0, 0);;
      shooter.setFeedingPower(0);
      angleChanging.setMotorPower(0);
      intake.setPowerToMotors(0);
      
      shooter.shooterState = STATE.IDLE;
      angleChanging.angleState = STATE.IDLE;
    }, chassis, shooter, angleChanging, intake);
  }
}