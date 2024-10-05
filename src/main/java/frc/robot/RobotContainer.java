package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer implements Sendable {
  public static RobotContainer robotContainer;


  

  

  public RobotContainer() {
    robotContainer = this;

    configureBindings();




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
    throw new UnsupportedOperationException("Unimplemented method 'initSendable'");
  }
}
