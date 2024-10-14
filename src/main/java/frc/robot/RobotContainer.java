package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.utils.LogManager;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.commands.DriveCommand;
import frc.robot.chassis.subsystems.Chassis;

public class RobotContainer implements Sendable {
  public static RobotContainer robotContainer;
  private Boolean isRed = true;
  CommandXboxController commandController;
  public CommandXboxController commandController2;
  public Joystick gitar;

  public LogManager logManager = new LogManager();

  public Chassis chassis;

  public Command resetOdometry;

  private double angle = 0;

  private int id = 0;

  

  public RobotContainer() {
    robotContainer = this;
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    chassis = new Chassis();
    commandController = new CommandXboxController(0);
    chassis.setDefaultCommand(new DriveCommand(chassis, commandController));

    createCommands();
    
    SmartDashboard.putData("RobotContainer", this);
    configureBindings();
  }



  public void createCommands() {


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
    builder.addDoubleProperty("angle in degriees for test",this::getAngle, this::SetAngle);
    builder.addIntegerProperty("id for test",this::getid, this::setid);
  }

  private double getAngle(){
    return angle;
  }
  private void SetAngle(double angle){
    this.angle = angle;
  }

  private int getid(){
    return id;
  }
  private void setid(Long id){
    this.id = id.intValue();
    System.out.println(id);
  }


  private void configureBindings() {

    commandController.back().onTrue(resetOdometry);
   
}

 
   
  public Command getAutonomousCommand() {
    // return null;
    return new RunCommand(()-> chassis.setModulesSteerPosition(Rotation2d.fromDegrees(angle), 1), chassis);
  }
}
