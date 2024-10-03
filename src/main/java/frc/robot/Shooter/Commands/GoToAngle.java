// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.utils.LookUpTable;
import frc.robot.Shooter.utils.Ready;

import static frc.robot.Shooter.ShooterConstants.*;

public class GoToAngle extends Command {
  /** Creates a new setShooting. */
  private LookUpTable lookupTable;
  private double[][] testingData;
  private AngleChanger angleChanger;
  public double wantedAngle;
  private double testingAngle; 
  private double distance;
  private boolean isInWing = true;//to do
  public STATE state;
  public static boolean isAngleReady;

  

  public GoToAngle(AngleChanger angleChanger) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleChanger = angleChanger;
    isAngleReady = Ready.isAngleReady(wantedAngle);
    lookupTable = new LookUpTable(testingData);
    SmartDashboard.putData(this);
    addRequirements(angleChanger);
  }

  @Override
  public void initSendable(SendableBuilder builder){
      builder.addDoubleProperty("Angle", this::getWantedAngle, this::setWantedAngle);
      
  }

  public double getWantedAngle(){
      return this.testingAngle;
  }
  
  public void setWantedAngle(double testingAngle){
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
    if (!isInWing)
      state = STATE.DELIVERY;
    switch(state){
      case AMP:
          wantedAngle = AMP_ANGLE;
          break;
      
      case STAGE:
          wantedAngle = STAGE_ANGLE;
  
          break;

      case WING:
          wantedAngle = WING_ANGLE;
          break;

      case SPEAKER:
           distance = -1;
           double[] speakerLookUpTableData = lookupTable.get(distance);
           wantedAngle = speakerLookUpTableData[0];
          break;

     case DELIVERY:
           distance = -1;
           double[] deliveryLookUpTableData = lookupTable.get(distance);
           wantedAngle = deliveryLookUpTableData[0];
          break;

      case TESTING:
          wantedAngle = testingAngle;
          break;

      case IDLE:
          break;
    }
    
    angleChanger.goToAngle(wantedAngle);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleChanger.goToAngle(DEFULT_ANGLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}


// p = ks + kv*v + kv2 + v^2 + ka*a