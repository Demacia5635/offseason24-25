// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubSystem.TestSubSytem;

public class SetCanCoderClockwiseCommand extends Command {
  /** Creates a new SetCanCoderClockwiseCommand. */
  private TestSubSytem testSubSytem;
  private boolean boolDirection;
  public SetCanCoderClockwiseCommand(TestSubSytem testSubSytem, boolean boolDirection) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.testSubSytem = testSubSytem;
    this.boolDirection = boolDirection;
    addRequirements(testSubSytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    testSubSytem.setCanCoderClockwise(boolDirection);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
