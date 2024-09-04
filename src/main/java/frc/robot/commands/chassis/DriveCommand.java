package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.chassis.Chassis;

import static frc.robot.utils.Utils.*;

import static frc.robot.subsystems.chassis.ChassisConstants.*;

public class DriveCommand extends Command {
  private final Chassis chassis;
  private final CommandXboxController commandXboxController;

  private double direction;

  private boolean isRed;
  private boolean precisionDrive = false;

  private boolean isSeeNote = false;
  private boolean hasNote = false;
  private boolean hasVx = false;

  private AutoIntake autoIntake;

  public DriveCommand(Chassis chassis, CommandXboxController commandXboxController) {
    this.chassis = chassis;
    this.commandXboxController = commandXboxController;
    this.autoIntake = new AutoIntake(chassis, commandXboxController);
    addRequirements(chassis);
    commandXboxController.b().onTrue(new InstantCommand(() -> precisionDrive = !precisionDrive));

  }

  @Override
  public void initialize() {

  }


  //need to add intake
  private boolean hasNote(){
    return false;
  }

  //need to add intake
  private boolean isSeeNote(){
    return true;
  }

  @Override
  public void execute() {
    isRed = chassis.isRed();
    direction = isRed ? 1 : -1;
    
    double joyX = deadband(commandXboxController.getLeftY(), 0.1) * direction;
    double joyY = deadband(commandXboxController.getLeftX(), 0.1) * direction;
    double rot = -(deadband(commandXboxController.getRightTriggerAxis(), 0.1)
        - deadband(commandXboxController.getLeftTriggerAxis(), 0.1));

    double velX = Math.pow(joyX, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyX);
    double velY = Math.pow(joyY, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyY);
    double velRot = Math.pow(rot, 2) * MAX_OMEGA_VELOCITY * Math.signum(rot);

    hasVx = new Translation2d(velX, velY).rotateBy(chassis.getAngle().unaryMinus()).getX() >= 0;
    hasNote = hasNote();
    isSeeNote = isSeeNote();

    if (precisionDrive) {
      velX /= 4;
      velY /= 4;
      velRot /= 4;
    }

    if(hasVx && !hasNote && isSeeNote){
      autoIntake.schedule();
      return;
    }


    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);
    
    chassis.setVelocities(speeds);
  
  }

}
