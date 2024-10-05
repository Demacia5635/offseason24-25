// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterConstants.AMP_VAR;
import frc.robot.Shooter.ShooterConstants.SHOOTER_VEL;
import frc.robot.Shooter.ShooterConstants.STAGE_VAR;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.ShooterConstants.SUBWOFFER_VAR;
import frc.robot.Shooter.Subsystems.Shooter;
import frc.robot.Shooter.utils.LookUpTable;
import frc.robot.Shooter.utils.Ready;

import static frc.robot.Shooter.ShooterConstants.*;

public class Shoot extends Command {
  
  private Shooter shooter;

  private double upMotorVelocity;
  private double downMotorVelocity;
  private double testingUpMotorVelocity;
  private double testingDownMotorVelocity;
  public STATE state;
  private double distance;
  private boolean isInWing;//to do
  private boolean isBetweenWing;
  private boolean isBetweenRivalWing;
  public boolean isReady;
  public boolean isfinished;

  private LookUpTable lookupTable;
  private double[][] testingData;

  private Timer shooterTimer;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter) {
    lookupTable = new LookUpTable(testingData);
    this.shooter = shooter;
    shooterTimer = new Timer();
    isReady = false;
    isfinished = false;
    SmartDashboard.putData(this);
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.ll
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("testingUpMotorVelocity", this::getUpMotorVelocity, this::setUpMotorVelocity);
    builder.addDoubleProperty("testingDownMotorVelocity", this::getDownMotorVelocity, this::setDownMotorVelocity);
  }

    public double getUpMotorVelocity(){
      return this.testingUpMotorVelocity;
    }
  
    public void setUpMotorVelocity(double testingUpMotorVelocity){
      this.testingUpMotorVelocity = testingUpMotorVelocity;
    }

    public double getDownMotorVelocity(){
      return this.testingDownMotorVelocity;
    }
  
    public void setDownMotorVelocity(double testingDownMotorVelocity){
      this.testingDownMotorVelocity = testingDownMotorVelocity;
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isBetweenWing){
      state = STATE.DELIVERY_MID;
    }
    switch(state){
      case AMP:
          upMotorVelocity = AMP_VAR.MOTOR_UP_AMP_VELOCITY;
          downMotorVelocity = AMP_VAR.MOTOR_DOWN_AMP_VELOCITY;
          break;

      case STAGE:
          upMotorVelocity = STAGE_VAR.MOTOR_UP_STAGE_VELOCITY;
          downMotorVelocity = STAGE_VAR.MOTOR_DOWN_STAGE_VELOCITY;
          break;

      case SUBWOFFER:
          upMotorVelocity = SUBWOFFER_VAR.MOTOR_UP_SUBWOFFER_VELOCITY;
          downMotorVelocity = SUBWOFFER_VAR.MOTOR_DOWN_SUBWOFFER_VELOCITY;
          break;

      case SPEAKER:
        distance = -1;
         double[] lookUpTableData = lookupTable.get(distance);
         upMotorVelocity = lookUpTableData[1];
         downMotorVelocity = lookUpTableData[2];
        break;

     case TESTING:
          upMotorVelocity = testingUpMotorVelocity;
          downMotorVelocity = testingDownMotorVelocity;
          break;

      case DELIVERY_MID:
          distance = -1;
          double[] deliveryMisLookUpTableData = lookupTable.get(distance);
          upMotorVelocity = deliveryMisLookUpTableData[1];
          downMotorVelocity = deliveryMisLookUpTableData[2];
          break;

      case DELIVERY_RIVAL:
          distance = -1;
          double[] deliveryRivalLookUpTableData = lookupTable.get(distance);
          upMotorVelocity = deliveryRivalLookUpTableData[1];
          downMotorVelocity = deliveryRivalLookUpTableData[2];
          break;

      case IDLE:
          break;
    }
    
    shooter.pidMotorVelocity(upMotorVelocity, downMotorVelocity);

    isReady = Ready.isReady(upMotorVelocity, downMotorVelocity, state)
        || isShooterReady;
    if (isReady){
      shooter.setFeedingPower(SHOOTER_VEL.FEEDING_MOTOR_POWER);
      isReady = false;
      isShooterReady = false;
      if (Shooter.tempIRSensor){
        shooterTimer.start();
      }
      while (shooterTimer.get()/1000 <= MIL_SEC_TO_SHOOT){}
        shooterTimer.stop();
        isfinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setMotorPower(0, 0);
    shooter.setFeedingPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfinished;
  }
}
