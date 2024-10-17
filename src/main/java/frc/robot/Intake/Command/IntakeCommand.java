package frc.robot.Intake.Command;

import static frc.robot.Intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Intake.IntakeConstants.NotePosition;
import frc.robot.Intake.Subsystem.Intake;

/** The command that feeds the note to the robot */
public class IntakeCommand extends Command {

  /** The intake subsistem */
  private Intake intake;

  Timer timer;
  Timer timerIntake;
  boolean hasTaken;

  /**
   * Takes the intake subsystem
   * @param intake
   */
  public IntakeCommand(Intake intake) {
    this.intake = intake;
    timer = new Timer();
    timerIntake = new Timer();
    hasTaken = false;

    addRequirements(intake);
  }

  /**
   * Sets to the default value to no note
   */
  @Override
  public void initialize(){
    intake.currentPosition = NotePosition.NO_NOTE;
    timer.start();
    hasTaken = false;
  }


  /**
   * Case for no note in intake, set motors to almost max
   * <br></br> case for note touching first motor pic up , Set motors to FIRST_TOUCH powers
   */
  @Override
  public void execute() {
    intake.setPowerToMotors(intake.currentPosition.power);

    if(intake.AmperHighMotorPickUp() && !hasTaken){
      intake.currentPosition = NotePosition.FIRST_TOUCH; 
      timerIntake.start();
      hasTaken = true;
    }

    if (intake.isNote() && !hasTaken) {
      timerIntake.start();
      hasTaken = true;
    }
  }

  /**
   * When the funcation end we updated the boolean vlaue 
   * <br></br> of is note in intake and turn the motors to 0
   */
  @Override
  public void end(boolean interrupted) {
    // if (!interrupted) {
    //   intake.isNoteInIntake = true;
    // }
    intake.setPowerToMotors(0);
    timer.stop();
    timer.reset();
    timerIntake.stop();
    timerIntake.reset();
    hasTaken = false;
  }

  /**
   * Case when the note is in the intake
   */
  @Override
  public boolean isFinished() {
    return timerIntake.get()*1000 > STOP_AFTER_NOTE || timer.get()*1000 > STOP_COMMAND_TIME;
  }
}
