// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.utils.LookupTable;

import static frc.robot.Shooter.ShooterConstants.*;

public class GoToAngle extends Command {
  /** Creates a new setShooting. */
  private LookupTable lookupTable;
  private double[][] arr;
  private AngleChanger angleChanging;
  public static double angle;
  private double distance;
  private double upMotorVelocity;
  private double downMotorVelocity;
  public static boolean isReady;
  public boolean isfinished;
  public STATE state;

  

  public GoToAngle(AngleChanger angleChanging) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleChanging = angleChanging;
    lookupTable = new LookupTable(arr);
    addRequirements(angleChanging);
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

      case SPEAKER:
           distance = -1;
           double[] lookUpTableData = lookupTable.get(distance);
           angle = lookUpTableData[0];
          break;

    /*TODO getting data from Look Up Table */
     case DELIVERY:
          angle = -1;
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