
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.leds.LedManager;
import frc.robot.leds.LedStrip;
import frc.robot.leds.strips.RSLStrip;
import frc.robot.leds.strips.MainLeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.vision.subsystem.VisionByTag;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Intake.Command.IntakeCommand;
import frc.robot.Intake.Subsystem.Intake;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Commands.Calibration;
import frc.robot.Shooter.Commands.GoToAngle;
import frc.robot.Shooter.Commands.Shoot;
import frc.robot.Shooter.Commands.WaitUntilShooterReady;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.Subsystems.Shooter;
import frc.robot.Shooter.utils.Ready;
import frc.robot.auto.ShootAndLeave;
import frc.robot.chassis.commands.DriveCommand;
import frc.robot.chassis.commands.DriveToNote;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;
import frc.robot.utils.Utils;

import static frc.robot.Constants.*;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer implements Sendable{
  public static RobotContainer robotContainer;
  public static Boolean isRed = false;
  CommandXboxController driverController;
  CommandXboxController operatorController;

  public LogManager logManager = new LogManager();

  public static Chassis chassis;
  public static Shooter shooter;
  public static AngleChanger angleChanging;
  public static Intake intake;

  private static LedManager ledManager;
  private MainLeds mainLeds;

  public IntakeCommand intakeCommand;
  public Shoot shootCommand;
  public Command resetOdometry;
  public DriveCommand driveCommand;
  public GoToAngle gotoAngleCommand;
  public Calibration calibration;
  public Command driveToNote;
  public WaitUntilShooterReady waitUntilShooterReady;

  public static boolean isDriverOverwriteShooter = false;
  public static boolean isTurningToSpeaker = true;

  Pigeon2 gyro;

  VisionByTag pose;
  Field2d field;

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {

    robotContainer = this;
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);
    operatorController = new CommandXboxController(1);

    chassis = new Chassis();
    shooter = new Shooter();
    angleChanging = new AngleChanger();
    intake = new Intake();
    ledManager = new LedManager();
    
    mainLeds = new MainLeds("main leds", ledManager, intake, chassis, shooter);
    

    calibration = new Calibration(angleChanging);
    shootCommand = new Shoot(shooter, intake, chassis);
    intakeCommand = new IntakeCommand(intake, shooter);
    resetOdometry = new InstantCommand(()-> chassis.setOdometryToForward())
                        .ignoringDisable(true);
    driveCommand = new DriveCommand(chassis, driverController);
    gotoAngleCommand = new GoToAngle(angleChanging, chassis);
    waitUntilShooterReady = new WaitUntilShooterReady(shooter);
   // driveToNote = new DriveToNote(chassis, 1.6, true).raceWith(intakeCommand); 

    chassis.setDefaultCommand(driveCommand);
   
    angleChanging.setDefaultCommand(gotoAngleCommand);
    

    NamedCommands.registerCommand("shoot", shootCommand);
    NamedCommands.registerCommand("intake", intakeCommand);
//    NamedCommands.registerCommand("goToAngle", gotoAngleCommand);
//    NamedCommands.registerCommand("calibration", calibration);
    NamedCommands.registerCommand("shooterReady", waitUntilShooterReady);

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("RobotContainer", this);
    SmartDashboard.putData("fiset gyro", new InstantCommand(()->gyro.setYaw(0)));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }


  private void configureBindings() {
    driverController.b().onTrue(new InstantCommand(()->{
      mainLeds.amp();
      angleChanging.angleState = STATE.DELIVERY;
      shooter.shooterState = STATE.DELIVERY;
    }));
    
    driverController.y().onTrue(new InstantCommand(()-> {
      mainLeds.autoIntake();
      driveCommand.setAutoIntake();
    }));
    driverController.a().onTrue(new IntakeCommand(intake, shooter));
    driverController.x().onTrue(shootCommand);
    driverController.rightBumper().onTrue(new InstantCommand(()->isDriverOverwriteShooter = true));
    driverController.rightStick().onTrue(new InstantCommand(()->{
      mainLeds.amp();
      angleChanging.angleState = STATE.SPEAKER;
      shooter.shooterState = STATE.SPEAKER;
    }));
    
    driverController.start().onTrue(new InstantCommand(()-> {
      mainLeds.amp();
      angleChanging.angleState = STATE.AMP;
      shooter.shooterState = STATE.AMP;
    }).ignoringDisable(true));

    driverController.povRight().onTrue(new InstantCommand(()->driveCommand.setPrecision()));
    driverController.povUp().onTrue(new InstantCommand(()->{
      mainLeds.amp();
      angleChanging.angleState = STATE.SUBWOFFER;
      shooter.shooterState = STATE.SUBWOFFER;
    }));

    driverController.leftBumper().onTrue(stopAll());
    
    driverController.povDown().onTrue(new InstantCommand(()-> driveCommand.rototeToAmp()));
    
    operatorController.a().onTrue(new RunCommand(()-> {
      intake.setPowerToMotors(-1);
      intake.isNoteInIntake = false;
    }, intake));

    operatorController.x().onTrue(new InstantCommand(()-> {
      shooter.setMotorPower(0.5, 0.5);
      shooter.setFeedingPower(1);
    }, shooter));

    operatorController.y().onTrue(new InstantCommand(()-> 
      Utils.setPipeline(Utils.getPipeline() == 0 ? 1 : 0))
      .ignoringDisable(true));

    operatorController.b().onTrue(calibration);
    
    operatorController.povUp().onTrue(new InstantCommand(()-> {
      intake.setPowerToMotors(0);
    }, intake));
    
    operatorController.povRight().onTrue(new InstantCommand(()-> {
      chassis.stop();
    }, chassis));

    operatorController.povDown().onTrue(new InstantCommand(()-> {
      shooter.setMotorPower(0, 0);
      shooter.setFeedingPower(0);
    }, shooter));

    operatorController.povLeft().onTrue(new InstantCommand(()-> {
      angleChanging.setMotorPower(0);
    }, angleChanging));

    operatorController.rightBumper().onTrue(new InstantCommand(()-> {
      shooter.shooterState = STATE.TESTING;
      angleChanging.angleState = STATE.TESTING;
    }));

    operatorController.leftBumper().onTrue(stopAll());

    operatorController.leftStick().onTrue(new InstantCommand(()-> 
    isTurningToSpeaker = !isTurningToSpeaker));

    operatorController.rightStick().onTrue(new InstantCommand(()-> intake.isNoteInIntake = !intake.isNoteInIntake).ignoringDisable(true));
    // operatorController.rightStick().onTrue(new InstantCommand(()-> mainLeds.operatorNote()));
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
    // return calibration;
    return new ShootAndLeave(chassis, shooter, angleChanging, intake);
  }

  // public Command calibration() {
  //   return calibration; // .andThen(new InstantCommand(()->angleChanging.setDefaultCommand(gotoAngleCommand)));
  // }

  // public void setAngleChangeCmd() {
  //   angleChanging.setDefaultCommand(gotoAngleCommand);
  // }

  public Command stopAll() {
    return new InstantCommand(()-> {
      shooter.setMotorPower(0, 0);;
      shooter.setFeedingPower(0);
      angleChanging.setMotorPower(0);
      intake.setPowerToMotors(0);
      
      // shooter.shooterState = STATE.IDLE;
      // angleChanging.angleState = STATE.IDLE;
    }, shooter, angleChanging, intake);
  }
}