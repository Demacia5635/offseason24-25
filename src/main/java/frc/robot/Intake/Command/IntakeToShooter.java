package frc.robot.Intake.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Subsystem.Intake;

public class IntakeToShooter extends Command {

  Intake intake;//takes the intake subsystem
  public IntakeToShooter(Intake intake) {
    this.intake = intake;
    addRequirements(intake);

  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.motorMoveSetPower(1);
  }

  @Override
  public void end(boolean interrupted) {
    Intake.isNoteInIntake = false;
    intake.setPowerToMotors(0.0);
  }


  @Override
  public boolean isFinished() {
    return intake.isNote();
  }
}
