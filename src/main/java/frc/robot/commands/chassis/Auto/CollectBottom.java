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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Field;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.utils.Utils;

public class CollectBottom extends Command {
  SequentialCommandGroup cmd;
  // pathPoint escapePoint1 = offset(Field.WingNotes[2], 0,-3.5, 0, 0.5);
    pathPoint escapePoint1 = offset(Field.CenterNotes[3], -3,-2, 20);

  pathPoint escapePoint2 = offset(Field.CenterNotes[4], -2,1, 0);
 // pathPoint escapePoint3 = offset(Field.CenterNotes[3], -0.5,1, 0);

  pathPoint shootPointFirst = new pathPoint(new Translation2d(3.18, 2.8), Rotation2d.fromDegrees(-40));
  pathPoint shootPoint = offset(Field.Speaker, 3,-1.5, -30);
  pathPoint shootPointAnchor = new pathPoint(new Translation2d(3.3, 2), Rotation2d.fromDegrees(-40), 0.4);

  Timer timer = new Timer();

  /** Creates a new CollectTop. */
  public CollectBottom() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    cmd = new SequentialCommandGroup(goTo(shootPointFirst, 1.5, true));
    //addCommands(turnToSpeakerAndWaitForReady(),cmd);
    // addCommands(new WaitCommand(0.7), cmd);
    addCommands(new WaitCommand(1), cmd);
    addCommands(shoot(1.5),cmd);
//    addCommands(goToMultiple(
//        new pathPoint[]{dummyPoint,escapePoint1,escapePoint2},2.5), cmd);
    addCommands(goTo(escapePoint2,1.5), cmd);
    addCommands(takeNote(), cmd);
    //addCommands(goToMultiple(new pathPoint[]{dummyPoint, shootPointAnchor, shootPoint}, 3.2), cmd);
    addCommands(goTo(shootPointFirst,1.5, true), cmd);

    //addCommands(turnToSpeakerAndWaitForReady(), cmd);
    addCommands(new WaitCommand(1), cmd);

    addCommands(shoot(1), cmd);
    addCommands(goTo(escapePoint1,1.5), cmd);

    // addCommands(goToMultiple(
    //     new pathPoint[]{dummyPoint,escapePoint1,escapePoint3},2.5), cmd);
    addCommands(takeNote(), cmd);
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
