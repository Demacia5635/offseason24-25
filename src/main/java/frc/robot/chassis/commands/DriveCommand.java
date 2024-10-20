package frc.robot.chassis.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.chassis.ChassisConstants.MAX_DRIVE_VELOCITY;
import static frc.robot.chassis.ChassisConstants.MAX_OMEGA_VELOCITY;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Shooter.Commands.Shoot;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.chassis.ChassisConstants;
import frc.robot.chassis.subsystems.Chassis;

import static frc.robot.utils.Utils.deadband;

public class DriveCommand extends Command {
  private final Chassis chassis;
  private final CommandXboxController commandXboxController;

  private double direction;

  private boolean isRed;
  private boolean precisionDrive = false;

  private boolean hasVx = false;
  private boolean hasNote = false;
  private boolean isSeeNote = false;
  private boolean isAutoIntake = false;

  // Rotation2d wantedAngleApriltag = new Rotation2d();
  // boolean rotateToApriltag = false;
  // PIDController rotationPidController = new PIDController(0.03, 0, 0.0008);

  public DriveCommand(Chassis chassis, CommandXboxController commandXboxController) {
    this.chassis = chassis;
    this.commandXboxController = commandXboxController;
    addRequirements(chassis);
    //commandXboxController.b().onTrue(new InstantCommand(() -> precisionDrive = !precisionDrive));
    // commandXboxController.b().onTrue(new InstantCommand(()-> isAutoIntake = !isAutoIntake));
  }

  public void setAutoIntake(){
    this.isAutoIntake = !isAutoIntake;
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
    double rot = (deadband(commandXboxController.getRightTriggerAxis(), 0.03)
        - deadband(commandXboxController.getLeftTriggerAxis(), 0.03));

    double velX = Math.pow(joyX, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyX);
    double velY = Math.pow(joyY, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyY);
    double velRot = Math.pow(rot, 2) * MAX_OMEGA_VELOCITY * Math.signum(rot);

    hasVx = ChassisSpeeds.fromFieldRelativeSpeeds(chassis.getChassisSpeeds(), chassis.getAngle()).vxMetersPerSecond < 0;
    isSeeNote = chassis.visionByNote.getAngleToNote().getDegrees() != 0;
    hasNote = RobotContainer.intake.isNoteInIntake;
   
    if (precisionDrive) {
      velX /= 4;
      velY /= 4;
      velRot /= 4;
    }
    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);

    Command c = RobotContainer.shooter.getCurrentCommand();
    

    if(hasVx && !hasNote && isSeeNote && isAutoIntake){
      
      //if(!RobotContainer.robotContainer.intakeCommand.isScheduled() && isAutoIntake) RobotContainer.robotContainer.intakeCommand.schedule();
      
    
      double angle = chassis.visionByNote.getAngleToNote().getDegrees();
      double vectorAngle = angle*2;
      Translation2d robotToNote = new Translation2d(getV(), Rotation2d.fromDegrees(vectorAngle));
      chassis.setVelocitiesRobotRel(new ChassisSpeeds(robotToNote.getX(), robotToNote.getY(), 0)); 
    } 
    else {
      if (c != null && c instanceof Shoot && RobotContainer.shooter.shooterState == STATE.SPEAKER) {
        chassis.setVelocitiesRotateToSpeaker(speeds);
      }
      else{
        chassis.setVelocities(speeds);
      }
      }
    }

    public double getV(){
       return Math.min(Math.hypot(commandXboxController.getLeftX(), commandXboxController.getLeftY()) * ChassisConstants.MAX_DRIVE_VELOCITY, ChassisConstants.MAX_DRIVE_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        
      chassis.stop();
    }
  }


