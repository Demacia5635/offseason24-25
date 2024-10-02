// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.utils.LookUpTable;

import static frc.robot.Shooter.ShooterConstants.*;

public class GoToAngle extends Command {
  /** Creates a new setShooting. */
  private LookUpTable lookupTable;
  private double[][] testingData;
  private AngleChanger angleChanging;
  public static double angle;
  private double testingAngle; 
  private double distance;
  public boolean isfinished;
  public STATE state;

  

  public GoToAngle(AngleChanger angleChanging) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleChanging = angleChanging;
    lookupTable = new LookUpTable(testingData);
    SmartDashboard.putData(this);
    addRequirements(angleChanging);
  }

  @Override
  public void initSendable(SendableBuilder builder){
      builder.addDoubleProperty("Angle", this::getAngle, this::setAngle);
      
  }

  public double getAngle(){
      return this.testingAngle;
  }
  
  public void setAngle(double testingAngle){
      this.testingAngle = testingAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  
      /**index: 0 - distance, 1 - angle, 2 - up vel, 3 - down vel 
       * 
       * @param lookUpTableData index of angle - 0, up vel - 1, down vel - 2
       * 
      */
  @Override
  public void execute() {
    switch(state){
      case AMP:
          angle = AMP_ANGLE;
          break;
      
      case STAGE:
          angle = STAGE_ANGLE;
  
          break;

      case WING:
          angle = WING_ANGLE;
          break;

      case SPEAKER:
           distance = -1;
           double[] speakerLookUpTableData = lookupTable.get(distance);
           angle = speakerLookUpTableData[0];
          break;

     case DELIVERY:
          distance = -1;
           double[] deliveryLookUpTableData = lookupTable.get(distance);
           angle = deliveryLookUpTableData[0];
          break;

      case TESTING:
          angle = testingAngle;
          break;

      case IDLE:
          angle = -1;
          break;
    }
    
    angleChanging.MotionMagic(angle);


    //פונקציה שמגדירה את isFinished כtrue אחרי שהרובוט ירה
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleChanging.MotionMagic(DEFULT_ANGLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfinished;
  }


}


// p = ks + kv*v + kv2 + v^2 + ka*a