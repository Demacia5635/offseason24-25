package frc.robot.Intake.Command;

import static frc.robot.Intake.IntakeConstants.STOP_COMMAND_TIME;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Intake.IntakeConstants.NotePosition;
import frc.robot.Intake.Subsystem.Intake;

/** The command that feeds the note to the robot */
public class IntakeCommand extends Command {

  /** The intake subsistem */
  private Intake intake;

  Timer timer;

  /**
   * Takes the intake subsystem
   * @param intake
   */
  public IntakeCommand(Intake intake) {
    this.intake = intake;
    timer = new Timer();
    addRequirements(intake);
  }

  /**
   * Sets to the default value to no note
   */
  @Override
  public void initialize(){
    intake.currentPosition = NotePosition.NO_NOTE;
    timer.start();
  }


  /**
   * Case for no note in intake, set motors to almost max
   * <br></br> case for note touching first motor pic up , Set motors to FIRST_TOUCH powers
   */
  @Override
  public void execute() {
    intake.setPowerToMotors(intake.currentPosition.power);

    if(intake.AmperHighMotorPickUp()){
      intake.currentPosition = NotePosition.FIRST_TOUCH; 
    }
  }

  /**
   * When the funcation end we updated the boolean vlaue 
   * <br></br> of is note in intake and turn the motors to 0
   */
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      intake.isNoteInIntake = true;
    }
    intake.setPowerToMotors(0);
    timer.stop();
    timer.reset();
  }

  /**
   * Case when the note is in the intake
   */
  @Override
  public boolean isFinished() {
    return intake.AmperHighMotorMove() ||  intake.isNoteInIntake || intake.isNote() /*|| timer.get()/1000 > STOP_COMMAND_TIME*/;
  }
}
