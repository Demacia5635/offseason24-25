package frc.robot.chassis.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.chassis.ChassisConstants.MAX_DRIVE_VELOCITY;
import static frc.robot.chassis.ChassisConstants.MAX_OMEGA_VELOCITY;

import frc.robot.RobotContainer;
import frc.robot.Intake.Subsystem.Intake;
import frc.robot.Shooter.Commands.Shoot;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.vision.subsystem.VisionByNote;

import static frc.robot.utils.Utils.deadband;

public class DriveCommand extends Command {
  private final Chassis chassis;
  VisionByNote note;
  Intake intake;
  private final CommandXboxController commandXboxController;

  private double direction;

  private boolean isRed;
  private boolean precisionDrive = false;

  private double continu = -1;
  private boolean isAutoIntake = false;

  // Rotation2d wantedAngleApriltag = new Rotation2d();
  // boolean rotateToApriltag = false;
  // PIDController rotationPidController = new PIDController(0.03, 0, 0.0008);

  public DriveCommand(Chassis chassis, CommandXboxController commandXboxController) {
    this.chassis = chassis;
    note = chassis.visionByNote;
    intake = RobotContainer.intake;
    this.commandXboxController = commandXboxController;
    addRequirements(chassis);
    // commandXboxController.b().onTrue(new InstantCommand(() -> precisionDrive =
    // !precisionDrive));
    // commandXboxController.b().onTrue(new InstantCommand(()-> isAutoIntake =
    // !isAutoIntake));
  }

  public void setAutoIntake() {
    this.isAutoIntake = !isAutoIntake;
  }

  public void setPrecision(){
    this.precisionDrive = !precisionDrive;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    isRed = chassis.isRed();
    direction = isRed ? 1 : -1;

    double joyX = deadband(commandXboxController.getLeftY(), 0.13) * direction;
    double joyY = deadband(commandXboxController.getLeftX(), 0.13) * direction;
    double rot = (deadband(commandXboxController.getRightTriggerAxis(), 0.003)
        - deadband(commandXboxController.getLeftTriggerAxis(), 0.003));

    double velX = Math.pow(joyX, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyX);
    double velY = Math.pow(joyY, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyY);
    double velRot = -rot * MAX_OMEGA_VELOCITY;


    if (precisionDrive) {
      velX /= 4;
      velY /= 4;
      velRot /= 4;
    }
    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);
    ChassisSpeeds robotSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, chassis.getAngle());

    // LogManager.log("-----------------------\nvx=" + 
    //     robotSpeed.vxMetersPerSecond + 
    //     " isNote " + !intake.isNote() + 
    //     " see=" + note.seeNote() + 
    //     " auto=" + isAutoIntake + 
    //     "\n------------------------------");
    if ((robotSpeed.vxMetersPerSecond < 0 &&
        !intake.isNote() &&
        note.seeNote() &&
        isAutoIntake)) {
      
      chassis.setAutoIntake(true);
      double angle = note.getNoteYaw();
      double vectorAngle = angle * 2;
      double v = Math.hypot(velX, velY);
      Translation2d robotToNote = new Translation2d(v, Rotation2d.fromDegrees(vectorAngle));
      chassis.setVelocitiesRobotRel(new ChassisSpeeds(robotToNote.getX(), robotToNote.getY(), 0));
        
      RobotContainer.robotContainer.intakeCommand.schedule();
      continu = 50;
    } 
    else if(continu>0 && !note.seeNote()){
      chassis.setAutoIntake(true);
      double vectorAngle = 0;
      double v = Math.hypot(velX, velY);
      Translation2d robotToNote = new Translation2d(v, Rotation2d.fromDegrees(vectorAngle));
      chassis.setVelocitiesRobotRel(new ChassisSpeeds(robotToNote.getX(), robotToNote.getY(), 0));
      continu --;
    }
    else {
      chassis.setAutoIntake(false);
      Command c = RobotContainer.shooter.getCurrentCommand();
      if (c != null && c instanceof Shoot && RobotContainer.shooter.shooterState == STATE.SPEAKER) {
        chassis.setVelocitiesRotateToSpeaker(speeds);
      } else {
        chassis.setVelocities(speeds);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {

    chassis.stop();
  }
}
