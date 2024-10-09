package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.commands.DriveCommand;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;
import frc.robot.utils.Logger;

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
  public double num = 0;
  

  

  public RobotContainer() {
    Logger.addTranslation2d("test", () -> new Translation2d(2, -3));
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
  public double get(){
    return num;
  }
  public void set(double num){
    this.num = num;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is Red",this::isRed, this::isRed);
    builder.addDoubleProperty("num",this::get, this:: set);
  }


  private void configureBindings() {
  
    //commandController.b().onTrue(new RunCommand(()->chassis.setGyroAngle(0)));
  
}

   
  public Command getAutonomousCommand() {
    return null;
    //return new RunCommand(() -> chassis.setModuleSteerVelocity(num, 0), chassis);
    //return new RunCommand(()->chassis.setModulesSteerPower(num));
    //return new RunCommand(()->chassis.setModulesSteerPosition(Rotation2d.fromDegrees(num), 0), chassis);
    // return new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0, 0, 1)), chassis)
    // .withTimeout(2).andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0, 0, -1))));
    //return new RunCommand(()->chassis.setModulesSteerVoltage(num, 0), chassis);
    //return new RunCommand(()->chassis.setModulesSteerPower(num, 2), chassis);

    // return new RunCommand(() -> {
    //   chassis.setModulesSteerPosition(0.0,0);
    //   chassis.setModulesSteerPosition(0.0,1);
    //   chassis.setModulesSteerPosition(0.0,2);
    //   chassis.setModulesSteerPosition(0.0,3);
    // }, chassis);P
    // return new RunCommand(()-> chassis.setModulesPower(0.1), chassis);
    //return new RunCommand(() -> chassis.speen(1), chassis);
  }
}
