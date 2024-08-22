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

public class Shoot extends Command {
  SequentialCommandGroup cmd;


  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd = new SequentialCommandGroup(new InstantCommand());
     addCommands(turnToSpeakerAndWaitForReady(), cmd);
     addCommands(new WaitCommand(0.5), cmd);
     addCommands(shoot(0.4), cmd);
    

    cmd.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd.isFinished();
  }
  
}

