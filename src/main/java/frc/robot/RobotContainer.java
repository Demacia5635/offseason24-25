// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.SubSystem.TestSubSytem;
import frc.robot.commands.ResetPigeonCommand;
import frc.robot.commands.SetCanCoderClockwiseCommand;
import frc.robot.commands.SetOffsetCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private CommandXboxController controller;
  private TestSubSytem testSubSytem;
  private Command ResetPigeonCommand;
  private Command SetOffsetCommand;
  private Command SetCanCoderClockwiseCommand;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    controller = new CommandXboxController(0);
    testSubSytem = new TestSubSytem(CANCODERID, CANCODERCANBUS, PIGEONID, PIGEONCANBUS);
    configureBindings();
  }

  //InstantCommand cmd = new InstantCommand();
  

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
    ResetPigeonCommand = new ResetPigeonCommand(testSubSytem);
    SetOffsetCommand = new SetOffsetCommand(testSubSytem, OFFSET);
    SetCanCoderClockwiseCommand = new SetCanCoderClockwiseCommand(testSubSytem, BOOLDIRECTION);
    controller.a().onTrue(ResetPigeonCommand);
    controller.b().onTrue(SetOffsetCommand);
    controller.x().onTrue(SetCanCoderClockwiseCommand);
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
