// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.Subsystems.Shooter;
import static frc.robot.Shooter.ShooterConstants.*;

public class GoToAngle extends Command {
  /** Creates a new setShooting. */

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
    addRequirements(angleChanging);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state){
      case AMP:
          angle = AMP_ANGLE;
          upMotorVelocity = MOTOR_UP_AMP_VELOCITY;
          downMotorVelocity = MOTOR_DOWN_AMP_VELOCITY;
          break;

      case SPEAKER:
          //distance = ?;
          //angle = ?;
          //upMotorVelocity = ?;
          //downMotorVelocity = ?;
          break;

     case DELIVERY:
          angle = DELIVERY_ANGLE;
          upMotorVelocity = MOTOR_UP_DELIVERY_VELOCITY;
          downMotorVelocity = MOTOR_DOWN_DELIVERY_VELOCITY;
          break;

      case IDLE:
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