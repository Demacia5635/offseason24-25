package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_MODE;
import frc.robot.subsystems.vision.VisionLimelight;
import frc.robot.utils.Utils;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.AutoPrevious.Shoot;
import frc.robot.commands.chassis.Auto.CollectBottom;
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
    vision = new VisionLimelight(chassis, chassis.getSwerveDrivePoseEstimator());
    intake = new Intake();
    // amp = new Amp();
    gitar = new Joystick(2);
    commandController2 = new CommandXboxController(1);
    commandController = new CommandXboxController(0);
    shooter = new Shooter();
    chassis.setDefaultCommand(new DriveCommand(chassis, commandController));
    led = new LedControll(9, 110);
  
    shooter.setDefaultCommand(new ActivateShooter(shooter, intake, chassis));
    createCommands();
    
    SmartDashboard.putData("RobotContainer", this);
    configureBindings();
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

    resetOdometry = new InstantCommand(()-> chassis.setOdometryToForward()).ignoringDisable(true);
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

    




    Trigger overrideAuto = new Trigger(() -> Utils.joystickOutOfDeadband(commandController));

    commandController.y().onTrue(driveToNote);
    commandController.x().onTrue(activateShooter);
    // B - in Drive Command
    commandController.a().onTrue(manualIntake);
    commandController.rightBumper().onTrue(shoot);
    commandController.start().onTrue(activateAmp);
    commandController.back().onTrue(resetOdometry);
    commandController.leftBumper().onTrue(disableCommand);
   // commandController.pov(270).onTrue(new GoToAMP1());
    commandController.pov(0).onTrue(shooter.getActivateShooterSubwoofer());
    overrideAuto.onTrue(chassis.getDefaultCommand());

    //Operator Controller
    //b -> set odometry to vision
    //y -> Shooter from Subwofer
    //x -> Shooter from Look up table
    //a -> amp
    //rightBumper -> Shooter from Podium
    //leftBumper -> Take note bakewords
    //pov(down) -> puke from intake(remove stack note)
    //pov(up) -> puke from Shooter(remove stack note)
    //pov(right) -> exposer+1
    //pov(left) -> exposer-1
    Command driverB = new InstantCommand((()->vision.setResetOdo(true))).ignoringDisable(true);
    driverB.setName("Perlman B");
    commandController2.b().onTrue(driverB);
    commandController2.y().onTrue(shooter.getActivateShooterSubwoofer());
    commandController2.x().onTrue(activateShooter);
    commandController2.a().onTrue(activateAmp);
    commandController2.rightBumper().onTrue(activatePodium);
    Command PerlmanleftBumper = (new RunCommand(()->shooter.feedingSetPow(-0.5), shooter).withTimeout(0.2))
      .andThen(new InstantCommand(()->shooter.feedingSetPow(0), shooter));
    PerlmanleftBumper.setName("Perlman l Bumber");
    commandController2.leftBumper().onTrue(PerlmanleftBumper);
    Command Ppov0 = new InstantCommand(()-> {intake.setPower(1); shooter.feedingSetPow(1); shooter.setVel(10);}, shooter, intake);
    Ppov0.setName("P Pov0");
    commandController2.pov(0).whileTrue(Ppov0);
    Command Ppov180 = new RunCommand(()-> intake.setPower(-1), intake).withTimeout(0.3);
    Ppov180.setName("P Pov 180");
    commandController2.pov(180).onTrue(Ppov180);

    Command Ppov90 = new InstantCommand(() -> {Utils.setPipeline(Math.min(Utils.getPipeline() + 1, 6)); }).ignoringDisable(true);
    Ppov180.setName("P Pov 90");
    commandController2.pov(90).onTrue(Ppov90);
    commandController2.pov(270).onTrue(new InstantCommand(() -> {Utils.setPipeline(Math.max(Utils.getPipeline() - 1, 0)); }).ignoringDisable(true));

    JoystickButton button1 = new JoystickButton(gitar, 1);
    JoystickButton button2 = new JoystickButton(gitar, 2);
    JoystickButton button3 = new JoystickButton(gitar, 3);
    JoystickButton button4 = new JoystickButton(gitar, 4);
    JoystickButton button5 = new JoystickButton(gitar, 5);
    JoystickButton button6 = new JoystickButton(gitar, 6);
    JoystickButton button7 = new JoystickButton(gitar, 7);
    JoystickButton button8 = new JoystickButton(gitar, 8);

    button1.onTrue(shooter.getActivateShooterSubwoofer());
    button2.onTrue(activateShooter);
    button3.onTrue(activateAmp);
    button4.onTrue(activatePodium);
    button5.onTrue(driverB);
    button6.onTrue(PerlmanleftBumper);
    button7.onTrue(Ppov180);
    button8.onTrue(new InstantCommand(() -> {Utils.setPipeline(Utils.getPipeline() != 2 ? Utils.getPipeline() + 1 : 0); }).ignoringDisable(true));
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
