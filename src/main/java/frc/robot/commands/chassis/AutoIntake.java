package frc.robot.commands.chassis;

import static frc.robot.subsystems.chassis.ChassisConstants.COLLECT_OFFSET_METERS;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_DRIVE_VELOCITY;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_ACCELERATION;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_VELOCITY;
import static frc.robot.subsystems.chassis.ChassisConstants.METER_IN_CM;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.chassis.Chassis;
import edu.wpi.first.wpilibj.Timer;

public class AutoIntake extends Command {
  Chassis chassis;
  double velocity;

  private double[] llpython;
  private double angle;

  NetworkTableEntry llentry;
  ChassisSpeeds speed;

  private boolean hasNote;



  CommandXboxController controller;


  public AutoIntake(Chassis chassis, CommandXboxController controller) {
    this.chassis = chassis;
    this.controller = controller;
    hasNote = false;

    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    llentry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython");
    llpython = llentry.getDoubleArray(new double[8]);

  }

  @Override
  public void execute() {

    llpython = llentry.getDoubleArray(new double[8]);
    
    angle = llpython[1];
    double vectorAngle = angle * 2;
    Translation2d robotToNote = new Translation2d(getV(), Rotation2d.fromDegrees(vectorAngle));
    chassis.setVelocitiesRobotRel(new ChassisSpeeds(robotToNote.getX(), robotToNote.getY(), 0));


    

  }
  //need intake
  public boolean hasNote(){
    return false;
    //return hasNote;
  }
  private double getV(){
    return Math.min(Math.hypot(controller.getLeftX(), controller.getLeftY()) * ChassisConstants.MAX_DRIVE_VELOCITY, ChassisConstants.MAX_DRIVE_VELOCITY);
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return hasNote || controller.pov(0).getAsBoolean();
  }

}