// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;


import static frc.robot.Shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.Subsystems.Shooter;
import frc.robot.Shooter.utils.LookupTable;

public class Shoot extends Command {
  
  private Shooter shooter;
  private AngleChanger angleChanger;

  private double upMotorVelocity;
  private double downMotorVelocity;
  public STATE state;
  private double distance;

  private LookupTable lookupTable;
  private double[][] testingData;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter) {
    lookupTable = new LookupTable(testingData);
    angleChanger = new AngleChanger();
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state){
      case AMP:
          upMotorVelocity = MOTOR_UP_AMP_VELOCITY;
          downMotorVelocity = MOTOR_DOWN_AMP_VELOCITY;
          break;

      case SPEAKER:
           distance = -1;
           double[] lookUpTableData = lookupTable.get(distance);
           upMotorVelocity = lookUpTableData[1];
           downMotorVelocity = lookUpTableData[2];
          break;

     case DELIVERY:
          upMotorVelocity = -1;
          downMotorVelocity = -1;
          break;

      case IDLE:
          break;
    }

    shooter.pidMotorVelocity(upMotorVelocity);

    if (GoToAngle.angle == angleChanger.getShooterAngle()
        && upMotorVelocity == shooter.getUpMotorVel()
        && downMotorVelocity == shooter.getDownMotorVel()) /*|| (הנהג לחץ על קפתור)*/{

      GoToAngle.isReady = true;
    }
    if (GoToAngle.isReady){
      shooter.setFeedingPower(FEEDING_MOTOR_POWER);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
