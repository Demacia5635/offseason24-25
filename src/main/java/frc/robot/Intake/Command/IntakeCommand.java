package frc.robot.Intake.Command;

import static frc.robot.Intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Intake.IntakeConstants.NotePosition;
import frc.robot.Intake.Subsystem.Intake;
import frc.robot.Shooter.Subsystems.Shooter;

/** The command that feeds the note to the robot */
public class IntakeCommand extends Command {

  /** The intake subsistem */
  private Intake intake;
  private Shooter shooter;

  Timer timer;
  Timer timerIntake;
  boolean hasTaken;
  boolean touchedDownWheels = false;

  /**
   * Takes the intake subsystem
   * @param intake
   */
  public IntakeCommand(Intake intake, Shooter shooter) {
    this.intake = intake;
    this.shooter = shooter;
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
    timerIntake.reset();
    timerIntake.stop();
    hasTaken = false;
    touchedDownWheels = false;
  }


  /**
   * Case for no note in intake, set motors to almost max
   * <br></br> case for note touching first motor pic up , Set motors to FIRST_TOUCH powers
   */
  @Override
  public void execute() {
    
    if (intake.AmperHighMotorPickUp()) {
      intake.currentPosition = NotePosition.FIRST_TOUCH;
      touchedDownWheels = true;
    }
    if(touchedDownWheels && intake.AmperHighMotorPickUp2()) {
      intake.isNoteInIntake = true;
      timerIntake.reset();
      timerIntake.start();
      touchedDownWheels = false;
    }
    
//    if (intake.isNote()) {
//      intake.currentPosition = NotePosition.AFTER_SEEING_NOTE;
//      imerIntake.start();
//    }

    intake.motorPickUpSetPower(intake.currentPosition.pickUpPow);
    intake.motorMoveSetPower(intake.currentPosition.movePow);
  }

  /**
   * When the funcation end we updated the boolean vlaue 
   * <br></br> of is note in intake and turn the motors to 0
   */
  @Override
  public void end(boolean interrupted) {
    intake.setPowerToMotors(0);
    shooter.setFeedingPower(0);
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
