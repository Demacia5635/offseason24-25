// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import static frc.robot.chassis.ChassisConstants.MAX_DRIVE_VELOCITY;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Intake.Subsystem.Intake;
import frc.robot.Shooter.Commands.Calibration;
import frc.robot.Shooter.Commands.Shoot;
import frc.robot.Shooter.Commands.WaitUntilShooterReady;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.Subsystems.Shooter;
import frc.robot.chassis.subsystems.Chassis;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAndLeave extends SequentialCommandGroup {
  private final Chassis chassis;
  private final Shooter shooter;
  private final AngleChanger angleChanger;
  private final Intake intake;


  /** Creates a new ShootAndLeave. */
  public ShootAndLeave(Chassis chassis, Shooter shooter, AngleChanger angleChanger, Intake intake) {
    this.chassis = chassis;
    this.shooter = shooter;
    this.angleChanger = angleChanger;
    this.intake = intake;
    
    addCommands(
      new Calibration(angleChanger),
      new SequentialCommandGroup(
        new Shoot(shooter, intake, chassis),
        new WaitUntilShooterReady(shooter)
      ),
      new RunCommand(()-> {
        chassis.setVelocities(
          new ChassisSpeeds(RobotContainer.isRed ? -1 : 1, 0, RobotContainer.isRed ? -0.3 : 0.3)
        );
      }, chassis)
    );
  }
}
