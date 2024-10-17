package frc.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.chassis.ChassisConstants.MAX_DRIVE_VELOCITY;
import static frc.robot.chassis.ChassisConstants.MAX_OMEGA_VELOCITY;
import frc.robot.chassis.subsystems.Chassis;

import static frc.robot.utils.Utils.deadband;

public class DriveCommand extends Command {
  private final Chassis chassis;
  private final CommandXboxController commandXboxController;

  private double direction;

  private boolean isRed;
  private boolean precisionDrive = false;

  // Rotation2d wantedAngleApriltag = new Rotation2d();
  // boolean rotateToApriltag = false;
  // PIDController rotationPidController = new PIDController(0.03, 0, 0.0008);

  public DriveCommand(Chassis chassis, CommandXboxController commandXboxController) {
    this.chassis = chassis;
    this.commandXboxController = commandXboxController;
    addRequirements(chassis);
    commandXboxController.b().onTrue(new InstantCommand(() -> precisionDrive = !precisionDrive));
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
    double velRot = -Math.pow(rot, 2) * MAX_OMEGA_VELOCITY * Math.signum(rot);

    if (precisionDrive) {
      velX /= 4;
      velY /= 4;
      velRot /= 4;
    }
    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);
    if(velX != 0 || velY != 0 || velRot != 0)
      System.out.println("Chassis speeds: " + speeds);
    chassis.setVelocities(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        
      chassis.stop();
    }
  }


