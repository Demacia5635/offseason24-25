// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter.Subsystems.AngleChanging;
import frc.robot.Shooter.Subsystems.Shooter;
import static frc.robot.Shooter.ShooterConstants.*;

public class SetShootingAndShoot extends CommandBase {
  /** Creates a new setShooting. */
  private Shooter shooter;
  private AngleChanging angleChanging;
  private double angle;
  private double distance;
  private double upMotorVelocity;
  private double downMotorVelocity;
  private STATE anglerState;
  

  public SetShootingAndShoot(Shooter shooter, AngleChanging angleChanging) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.angleChanging = angleChanging;
    addRequirements(shooter);
    addRequirements(angleChanging);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(anglerState){
      case AMP:
        angle = AMP_ANGLE;
        
      
      case SPEAKER:
      

      case DELIVERY:
      

        

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setUpMotorPower(0);
    shooter.setDownMotorPower(0);
    angleChanging.setAngleChangingMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
