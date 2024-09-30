// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis.Auto;
import static frc.robot.commands.chassis.Auto.AutoUtils.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Field;
import frc.robot.PathFollow.Util.pathPoint;

public class DestroyCenter extends Command {
  SequentialCommandGroup cmd;
  Timer timer = new Timer();
  pathPoint centerNote1 = offset(Field.CenterNotes[0], 0,0,0);
  pathPoint centerNote5 = offset(Field.CenterNotes[4], 0,0.5,0);
  pathPoint shootPoint = new pathPoint(new Translation2d(2,3.9), Rotation2d.fromDegrees(-50));
  pathPoint firstPoint = new pathPoint(new Translation2d(2,3), new Rotation2d(),1);

  /** Creates a new CollectTop. */
  public DestroyCenter() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    cmd = new SequentialCommandGroup(goTo(shootPoint, 2, false, 2));
    addCommands(shoot(0), cmd);
    addCommands(goToMultiple(new pathPoint[]{dummyPoint, firstPoint, centerNote5},2),cmd);
    addCommands(goToRotate(centerNote1,2, 3), cmd);
    addCommands(new InstantCommand(()->System.out.println("\n\n\n finish - time = " + timer.get() + "\n\n\n")), cmd);

    cmd.schedule();
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
