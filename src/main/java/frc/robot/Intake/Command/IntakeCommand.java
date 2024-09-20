package frc.robot.Intake.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Intake.Intake;
import frc.robot.Intake.IntakeConstants;
import frc.robot.Intake.IntakeConstants.NotePosition;

import static frc.robot.Intake.IntakeConstants.*;
public class IntakeCommand extends CommandBase {

  private Intake intake;
  NotePosition currentPosition;

  public IntakeCommand(Intake intake) {
    this.currentPosition = NotePosition.NO_NOTE;// i know you dont have to put "this" but it helps me 
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if(intake.isAmperHighMotorDown()){
      currentPosition = NotePosition.FIRST_TOUCH;
    }
    else if(intake.isNote()){
      currentPosition = NotePosition.IR_SENSOR;
    }
    else if(intake.isAmperHighMotorUp()){
      currentPosition = NotePosition.SECOND_TOUCH; 

    }
    // else if(!(currentPosition.equals(currentPosition.NO_NOTE))){
    // }

    intake.setPower(NotePositionToVoltage.get(currentPosition));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return intake.isAmperHighMotorUp();
  }
}
