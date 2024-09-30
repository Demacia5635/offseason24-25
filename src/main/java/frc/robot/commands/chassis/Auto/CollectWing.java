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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Field;
import frc.robot.PathFollow.Util.pathPoint;

public class CollectWing extends Command {
  SequentialCommandGroup cmd;
  pathPoint wingNote1 = offset(Field.WingNotes[0], 0,0, 0);
  pathPoint wingNote2 = offset(Field.WingNotes[1], 0,0, 0);
  pathPoint wingNote3 = offset(Field.WingNotes[2], 0,0, 0);

  
  //pathPoint shootPoint = offset(Field.Speaker, 0,0,0);
  pathPoint firstShootPoint = new pathPoint(new Translation2d(1.7, 4.8), Rotation2d.fromDegrees(-10));
  pathPoint secondShootPoint = new pathPoint(new Translation2d(1.7, 5.3), Rotation2d.fromDegrees(0));
  pathPoint thirdShootPoint = new pathPoint(new Translation2d(1.8, 6.2), Rotation2d.fromDegrees(10));

  Timer timer = new Timer();


  /** Creates a new CollectTop. */
  public CollectWing() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    cmd = new SequentialCommandGroup(goTo(firstShootPoint,1.6, true));
    //addCommands(turnToSpeakerAndWaitForReady(), cmd);
    addCommands(new WaitCommand(0.5), cmd);
    addCommands(shoot(0.4), cmd);
    addCommands(takeNote(), cmd);
    addCommands(goTo(secondShootPoint,2.2, false), cmd);
    addCommands(turnToSpeakerAndWaitForReady(), cmd);
    addCommands(shoot(0.5), cmd);
    addCommands(takeNote(), cmd);
    addCommands(goTo(thirdShootPoint,2.2, false), cmd);
    addCommands(turnToSpeakerAndWaitForReady(), cmd);
    addCommands(shoot(0.5), cmd);
    addCommands(takeNote(), cmd);
    addCommands(goTo(thirdShootPoint,2.2,false), cmd);
    addCommands(turnToSpeakerAndWaitForReady(), cmd);
    addCommands(shoot(0), cmd);
    addCommands(new InstantCommand(()->System.out.println("\n\n\n finish - time = " + timer.get() + "\n\n\n")), cmd);

    cmd.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Collect Wing Time=" + timer.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd.isFinished();
  }
  
}
