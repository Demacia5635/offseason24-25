package frc.robot.Intake.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Intake.IntakeConstants;
import frc.robot.Intake.IntakeConstants.NotePosition;
import frc.robot.Intake.Subsystem.Intake;

import static frc.robot.Intake.IntakeConstants.*;
public class IntakeCommand extends Command {

  private Intake intake;

  public IntakeCommand(Intake intake) {
    

    this.intake = intake;
    
    Intake.currentPosition = 
    !IS_TESTING 
      ? NotePosition.NO_NOTE
      : NotePosition.TEST_NO_NOTE;

    addRequirements(intake);

  }



  @Override
  public void execute() {

    if(!Intake.isNoteInIntake){
      intake.setPowerToMotors(Intake.currentPosition.power);//gose to constractor of enum and gives back the power
      
      if(!IS_TESTING){

        if(intake.AmperHighMotorPickUp()){
          Intake.currentPosition = NotePosition.FIRST_TOUCH;
        }

      }

      else{   

        if(intake.AmperHighMotorMove()){
          Intake.currentPosition = NotePosition.TEST_FIRST_TOUCH;
        }

      }

    }

  }

  @Override
  public void end(boolean interrupted) {
    Intake.isNoteInIntake = true;
    intake.setPowerToMotors(0);
  }

  @Override
  public boolean isFinished() {
    return intake.AmperHighMotorMove() || Intake.isNoteInIntake || intake.isNote();
  }
}
