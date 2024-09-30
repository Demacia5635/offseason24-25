package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;


public class DriveAndPickNote extends ParallelRaceGroup {
  /** Auto drive to note and intake it. */
  public DriveAndPickNote(Chassis chassis, Intake intake) {

    addCommands(new DriveToNote(chassis, 1, true), new IntakeCommand(intake));
  }
}
