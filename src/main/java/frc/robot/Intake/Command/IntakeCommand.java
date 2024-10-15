package frc.robot.Intake.Command;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Intake.IntakeConstants.NotePosition;
import frc.robot.Intake.Subsystem.Intake;

/** The command that feeds the note to the robot */
public class IntakeCommand extends Command {

  /** The intake subsistem */
  private Intake intake;

  /**
   * Takes the intake subsystem
   * @param intake
   */
  public IntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  /**
   * Sets to the default value to no note
   */
  @Override
  public void initialize(){
    Intake.currentPosition = NotePosition.NO_NOTE;
  }


  /**
   * Case for no note in intake, set motors to almost max
   * <br></br> case for note touching first motor pic up , Set motors to FIRST_TOUCH powers
   */
  @Override
  public void execute() {

    if(!Intake.isNoteInIntake){
      intake.setPowerToMotors(Intake.currentPosition.power);
      if(intake.AmperHighMotorPickUp()){
        Intake.currentPosition = NotePosition.FIRST_TOUCH; 
      }
    }
  }

  /**
   * When the funcation end we updated the boolean vlaue 
   * <br></br> of is note in intake and turn the motors to 0
   */
  @Override
  public void end(boolean interrupted) {
    Intake.isNoteInIntake = true;
    intake.setPowerToMotors(0);
  }

  /**
   * Case when the note is in the intake
   */
  @Override
  public boolean isFinished() {
    return intake.AmperHighMotorMove() || Intake.isNoteInIntake || intake.isNote();
  }
}
