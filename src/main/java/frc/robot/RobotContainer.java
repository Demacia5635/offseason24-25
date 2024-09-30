// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Shooter.Commands.GoToAngle;
import frc.robot.Shooter.Commands.Shoot;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.Subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  private Shooter shooter;
  private AngleChanger angleChanging;
  public static boolean isShooterReady = false;
  int x;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    shooter = new Shooter();
    angleChanging = new AngleChanger();
    shooter.setDefaultCommand(new Shoot(shooter));
    angleChanging.setDefaultCommand(new GoToAngle(angleChanging));

    
    // Configure the trigger bindings
    configureBindings();
  }


  // public Trigger isAmp(){
  //   return m_driverController.a().onTrue(new GoToAngle_AndShoot(shooter, angleChanging));
  // }

  // public Trigger isDelivery(){
  //   return m_driverController.b().onTrue(new GoToAngle_AndShoot(shooter, angleChanging));
  // }

  //   public Trigger isSpeaker(){
  //   return m_driverController.x().onTrue(new GoToAngle_AndShoot(shooter, angleChanging));
  // }
 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    controller.a().onTrue(new InstantCommand(() -> {
      if (shooter.shooterState == STATE.SPEAKER){
        shooter.shooterState = STATE.AMP;
        angleChanging.angleState = STATE.AMP;
      }
      if (shooter.shooterState == STATE.AMP){
        shooter.shooterState = STATE.SPEAKER;
        angleChanging.angleState = STATE.SPEAKER;
      }
    }, shooter, angleChanging));
    controller.b().onTrue(new InstantCommand(() -> {
        isShooterReady = true;
    }, shooter));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
