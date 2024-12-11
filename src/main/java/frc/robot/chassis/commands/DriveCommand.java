package frc.robot.chassis.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.chassis.ChassisConstants.MAX_DRIVE_VELOCITY;
import static frc.robot.chassis.ChassisConstants.MAX_OMEGA_VELOCITY;

import frc.robot.RobotContainer;
import frc.robot.Intake.Subsystem.Intake;
import frc.robot.Shooter.Commands.Shoot;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;
import frc.robot.vision.subsystem.VisionByNote;

import static frc.robot.utils.Utils.deadband;

public class DriveCommand extends Command  implements Sendable{
  private final Chassis chassis;
  VisionByNote note;
  Intake intake;
  private final PS5Controller controller;

  private double direction;

  private boolean isRed;
  private boolean precisionDrive = false;

  private double continu = -1;
  private boolean isAutoIntake = false;
  private boolean isNoteInIntake = false;

  public boolean isRotateToMinus90 = false;
  Timer timerIsRotateToMinus90;

  // Rotation2d wantedAngleApriltag = new Rotation2d();
  // boolean rotateToApriltag = false;
  // PIDController rotationPidController = new PIDController(0.03, 0, 0.0008);

  public DriveCommand(Chassis chassis, PS5Controller controller) {
    this.chassis = chassis;
    note = chassis.visionByNote;
    intake = RobotContainer.intake;
    this.controller = controller;
    timerIsRotateToMinus90 = new Timer();
    addRequirements(chassis);
    // commandXboxController.b().onTrue(new InstantCommand(() -> precisionDrive =
    // !precisionDrive));
    // commandXboxController.b().onTrue(new InstantCommand(()-> isAutoIntake =
    // !isAutoIntake));
    SmartDashboard.putData(this);
  }

  public void rototeToAmp() {
    isRotateToMinus90 = true;
    timerIsRotateToMinus90.reset();
    timerIsRotateToMinus90.start();
  }

  public void setAutoIntake() {
    this.isAutoIntake = !isAutoIntake;
  }

  public void setPrecision(){
    this.precisionDrive = !precisionDrive;
  }

  @Override
  public void initialize() {
    isNoteInIntake = false;
    isRotateToMinus90 = false;
  }

  @Override
  public void execute() {
    isRed = chassis.isRed();
    direction = isRed ? 1 : -1;

    double joyX = deadband(controller.getLeftY(), 0.13) * direction;
    double joyY = deadband(controller.getLeftX(), 0.13) * direction;
    double rot = (deadband((controller.getR2Axis() + 1) / 2, 0.05)
    - deadband((controller.getL2Axis() + 1) / 2, 0.07));

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
        isAutoIntake &&
        !isNoteInIntake)) {
      
      chassis.setAutoIntake(true);
      double angle = note.getNoteYaw();
      double vectorAngle = angle * 2 + 180;
      double v = Math.hypot(velX, velY);
      if(v > 3) {
        v = 3;
      }
      Translation2d robotToNote = new Translation2d(v, Rotation2d.fromDegrees(vectorAngle));
      chassis.setVelocitiesRobotRel(new ChassisSpeeds(robotToNote.getX(), robotToNote.getY(), 0));
      
      RobotContainer.robotContainer.intakeCommand.schedule();
      isNoteInIntake = true;
      continu = 10;
    }
    else if(continu>0 && !intake.isNote()){
      chassis.setAutoIntake(true);
      isNoteInIntake = false;
      double vectorAngle = 180;
      double v = Math.hypot(velX, velY);
      Translation2d robotToNote = new Translation2d(v, Rotation2d.fromDegrees(vectorAngle));
      chassis.setVelocitiesRobotRel(new ChassisSpeeds(robotToNote.getX(), robotToNote.getY(), 0));
      continu --;
    }
    else {
      chassis.setAutoIntake(false);
      Command c = RobotContainer.shooter.getCurrentCommand();
      if (c != null && c instanceof Shoot && RobotContainer.shooter.shooterState == STATE.SPEAKER && RobotContainer.isTurningToSpeaker) {
        chassis.setVelocitiesRotateToSpeaker(speeds);
      } else {
        if (timerIsRotateToMinus90.hasElapsed(2)) {
          timerIsRotateToMinus90.stop();
          timerIsRotateToMinus90.reset();
          isRotateToMinus90 = false;
        } else if (isRotateToMinus90) {
          chassis.setVelocitiesRotateToAngle(speeds, Rotation2d.fromDegrees(180));
        } else {
          chassis.setVelocities(speeds);
        }
      }
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is auto intake", ()-> isAutoIntake, null);
  }

  @Override
  public void end(boolean interrupted) {

    chassis.stop();
  }
}
