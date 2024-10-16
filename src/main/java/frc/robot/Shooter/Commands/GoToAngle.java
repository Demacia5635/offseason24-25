// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterConstants.AMP_VAR;
import frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_VAR;
import frc.robot.Shooter.ShooterConstants.DISTANCES;
import frc.robot.Shooter.ShooterConstants.STAGE_VAR;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.ShooterConstants.SUBWOFFER_VAR;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.utils.LookUpTable;
import frc.robot.Shooter.utils.Ready;

import static frc.robot.Shooter.ShooterConstants.*;


public class GoToAngle extends Command {
  /** Creates a new setShooting. */
  private LookUpTable lookupTable;
  private AngleChanger angleChanger;
  public double wantedAngle = 35;
  private double testingAngle = 35; 
  private double distance = 0;
  public double XDistance = 0;

  public STATE state;
  public static boolean isAngleReady = false;

  

  public GoToAngle(AngleChanger angleChanger) {
    this.angleChanger = angleChanger;
    state = angleChanger.angleState;
    lookupTable = angleChanger.lookUpTable;
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
    if (XDistance >= DISTANCES.WING_DISTANCE && XDistance < DISTANCES.RIVAL_WING_DISTANCE && state != STATE.AMP && state != STATE.STAGE && state != STATE.SUBWOFFER && state != STATE.TESTING){
      state = STATE.DELIVERY_MID;
    }
    if (XDistance >= DISTANCES.RIVAL_WING_DISTANCE && state != STATE.AMP && state != STATE.STAGE && state != STATE.SUBWOFFER && state != STATE.TESTING){
      state = STATE.DELIVERY_RIVAL;
    }
    switch(state){
      case AMP:
          wantedAngle = AMP_VAR.AMP_ANGLE;
          break;
      
      case STAGE:
          wantedAngle = STAGE_VAR.STAGE_ANGLE;
  
          break;

      case SUBWOFFER:
          wantedAngle = SUBWOFFER_VAR.SUBWOFFER_ANGLE;
          break;

      case SPEAKER:
           distance = -1;
           double[] speakerLookUpTableData = lookupTable.get(distance);
           wantedAngle = speakerLookUpTableData[0];
          break;

     case DELIVERY_MID:
           distance = -1;
           double[] deliveryMisLookUpTableData = lookupTable.get(distance);
           wantedAngle = deliveryMisLookUpTableData[0];
          break;

     case DELIVERY_RIVAL:
           distance = -1;
           double[] deliveryRivalLookUpTableData = lookupTable.get(distance);
           wantedAngle = deliveryRivalLookUpTableData[0];
          break;

      case TESTING:
          wantedAngle = testingAngle;
          break;

      case IDLE:
        wantedAngle = IDLE_VAR.IDLE_ANGLE;
          break;
    }
    
    while (angleChanger.getAngle() <= ANGLE_CHANGING_VAR.MIN_ANGLE){
      angleChanger.setMotorPower(-ANGLE_CHANGING_POW.ANGLE_MOTOR_POWER);
    }
    while (angleChanger.getAngle() >= ANGLE_CHANGING_VAR.TOP_ANGLE){
      angleChanger.setMotorPower(ANGLE_CHANGING_POW.ANGLE_MOTOR_POWER);
    }

    isAngleReady = Ready.isAngleReady(wantedAngle);

    // angleChanger.goToAngle(wantedAngle);
    angleChanger.goToAnglePositionVol(wantedAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}


// p = ks + kv*v + kv2 + v^2 + ka*a