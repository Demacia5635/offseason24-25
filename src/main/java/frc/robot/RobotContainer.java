package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.vision.subsystem.visionByNote;
import frc.robot.vision.subsystem.visionByTag;

public class RobotContainer implements Sendable {
  public static RobotContainer robotContainer;


  Pigeon2 gyro;

  visionByTag pose;
  visionByNote note;
  Field2d field;

  public RobotContainer() {
    robotContainer = this;
    gyro = new Pigeon2(14);
    pose = new visionByTag(gyro);
    note = new visionByNote(pose.getRoobotPose());
    configureBindings();
    SmartDashboard.putData(this);
    SmartDashboard.putData("fiset gyro", new InstantCommand(()->gyro.setYaw(0)));





  }


  public void createCommands() {

}






  private void configureBindings() {
  
  
}

   
  public Command getAutonomousCommand() {
    return null;
  }


  @Override
  public void initSendable(SendableBuilder builder) {
  }
}
