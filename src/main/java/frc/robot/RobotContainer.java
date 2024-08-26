package frc.robot;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.commands.DriveCommand;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;

public class RobotContainer implements Sendable {
  public static RobotContainer robotContainer;
  private Boolean isRed = true;
  CommandXboxController commandController;
  public CommandXboxController commandController2;
  public Joystick gitar;


  public Chassis chassis;

  private LogManager LM = new LogManager();

  public Command shoot; // shoot to amp or to speaker
  public Command driveToNote;
  public Command manualIntake;
  public Command activateAmp;
  public Command disableCommand;
  public Command resetOdometry;
  public Command activatePodium;
  public Command activateShooter;
  public Command activateSubwoofer;
  

  

  public RobotContainer() {
    robotContainer = this;
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    chassis = new Chassis();
    
    commandController2 = new CommandXboxController(1);
    commandController = new CommandXboxController(0);
    chassis.setDefaultCommand(new DriveCommand(chassis, commandController));

      createCommands();
    
    SmartDashboard.putData("RobotContainer", this);
    configureBindings();
  }


  public void createCommands() {
    SmartDashboard.putNumber("VEL CALIBRATE", 0);

    SmartDashboard.putNumber("ANGLE CALIBRATE", 0);




    resetOdometry = new InstantCommand(()-> chassis.setOdometryToForward()).ignoringDisable(true);

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
  
}

   
  public Command getAutonomousCommand() {
    return null;
  }
}
