package frc.robot;

import static frc.robot.utils.Utils.angelErrorInRadians;
import static frc.robot.utils.Utils.speakerPosition;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.StringIdGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.commands.shooter.AngleCalibrate;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_MODE;
import frc.robot.subsystems.vision.VisionLimelight;
import frc.robot.utils.Utils;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.GoToAngleChassis;
import frc.robot.commands.chassis.AutoPrevious.Fowrard;
import frc.robot.commands.chassis.AutoPrevious.Shoot;
import frc.robot.commands.chassis.AutoPrevious.StartBottom1;
import frc.robot.commands.chassis.AutoPrevious.StartBottom2;
import frc.robot.commands.chassis.AutoPrevious.StartBottomEscape;
import frc.robot.commands.chassis.AutoPrevious.StartBottomPlayoffs;
import frc.robot.commands.chassis.AutoPrevious.StartMiddle1;
import frc.robot.commands.chassis.AutoPrevious.StartMiddle2;
import frc.robot.commands.chassis.AutoPrevious.StartTOP1;
import frc.robot.commands.chassis.AutoPrevious.StartTOP2;
import frc.robot.commands.chassis.Auto.AutoChooser;
import frc.robot.commands.chassis.Auto.CollectBottom;
import frc.robot.commands.chassis.Auto.CollectTop;
import frc.robot.commands.chassis.Auto.CollectWing;
import frc.robot.commands.chassis.Auto.DestroyCenter;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LedControll;

public class RobotContainer implements Sendable {
  public static RobotContainer robotContainer;
  private Boolean isRed = true;
  CommandXboxController commandController;
  public CommandXboxController commandController2;
  public Joystick gitar;

 
  public Shooter shooter;
  public Amp amp;
  public Intake intake;
  public Chassis chassis;
  public VisionLimelight vision;
  public LedControll led;

  public Command shoot; // shoot to amp or to speaker
  public Command driveToNote;
  public Command manualIntake;
  public Command activateAmp;
  public Command disableCommand;
  public Command resetOdometry;
  public Command activatePodium;
  public Command activateShooter;
  public Command activateSubwoofer;
  
  private enum AutoOptions { Shoot, Wing, Destroy, Bottom};
  private SendableChooser<AutoOptions> autoChoose;

  

  public RobotContainer() {
    robotContainer = this;
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    chassis = new Chassis();
    commandController2 = new CommandXboxController(1);
    commandController = new CommandXboxController(0);
   //.. shooter = new Shooter();
    chassis.setDefaultCommand(new DriveCommand(chassis, commandController));
   
  
   // shooter.setDefaultCommand(new ActivateShooter(shooter, intake, chassis));
    //createCommands();
    
    SmartDashboard.putData("RobotContainer", this);
    resetOdometry = new InstantCommand(()-> chassis.setOdometryToForward()).ignoringDisable(true);

  }

  public void stopAll() {
    shooter.stopAll();
    intake.stop();
  }

  public void createCommands() {
    SmartDashboard.putNumber("VEL CALIBRATE", 0);

    SmartDashboard.putNumber("ANGLE CALIBRATE", 0);



    driveToNote = new DriveToNote(chassis, 1.6, true).raceWith(new IntakeCommand(intake)).andThen(new IntakeCommand(intake));
    shoot = shooter.getShootCommand();
    activateShooter = shooter.getActivateShooterAuto();
    manualIntake = new IntakeCommand(intake);
    activateAmp = shooter.getActivateShooterToAmp();
    activatePodium = shooter.getActivateShooterToPodium();

        disableCommand = new InstantCommand(()-> stopAll(),intake);

    autoChoose = new SendableChooser<AutoOptions>();
    for(AutoOptions a : AutoOptions.values()) {
      autoChoose.addOption(a.name(), a);
    }
    SmartDashboard.putData("Auto Chooser", autoChoose);
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


  private void configureBindings() {
    // Buttons:
    //Driver controller
    // Y - Auto Intake
    // X - Activate Shooter
    // B - Drive precision mode
    // A - Manual Intake.
    // POV-UP - Shoot
    // POV-Down -
    // POV-Left - Go to Amp and shoot
    // POV-Right - Active shooter to shoot from subwaffer
    // LeftBumber - disable all
    //RightBumper - speaker from subwoofer
    // Back - chassie gyro set
    //Start - Amp

    




 
    commandController.back().onTrue(resetOdometry);
 }


  public void calibrateSetIdle() {
    shooter.stopAll();
  }

 
   
  public Command getAutonomousCommand() {

    AutoOptions autoOption = autoChoose.getSelected();
    Command cmd = null;
    switch(autoOption) {
      case Shoot:
        cmd = new Shoot();
        break;
      case Wing:
        cmd = new CollectWing();
        break;
      case Destroy:
        cmd = new DestroyCenter();
        break;
      case Bottom:
        cmd = new CollectBottom();
        break;
    }
    
    if(cmd != null) {
      return cmd.alongWith( 
        new InstantCommand(()->shooter.setShooterMode(SHOOTER_MODE.AUTO_CONTINIOUS))
        .andThen(new ActivateShooter(shooter, intake, chassis)));
    } else {
      return null;
    }
  }
}
