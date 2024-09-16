// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.subpose;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private subpose pose;
  private CommandXboxController controller;
  private Pigeon2 giro;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    pose = new subpose();
    configureBindings();

    giro = new Pigeon2(14);

    controller = new CommandXboxController(0);
  }




  private void configureBindings() {
    controller.a().onTrue(new InstantCommand(()->subpose.resetGiro(giro)));
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
