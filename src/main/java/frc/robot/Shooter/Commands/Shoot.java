// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.Intake.Subsystem.Intake;
import frc.robot.Shooter.ShooterConstants.AMP_VAR;
import frc.robot.Shooter.ShooterConstants.SHOOTER_POW;
import frc.robot.Shooter.ShooterConstants.STAGE_VAR;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.ShooterConstants.SUBWOFFER_VAR;
import frc.robot.Shooter.Subsystems.Shooter;
import frc.robot.Shooter.utils.LookUpTable;
import frc.robot.Shooter.utils.Ready;
import frc.robot.chassis.subsystems.Chassis;

import static frc.robot.Shooter.ShooterConstants.*;

public class Shoot extends Command {

  private Shooter shooter;
  private Intake intake;
  private Chassis chassis;

  private double upMotorVelocity;
  private double downMotorVelocity;
  private double testingUpMotorVelocity;
  private double testingDownMotorVelocity;

  public STATE state;
  private double distence;
  private double distenceX;

  public boolean isReady;
  public boolean isfinished;

  private LookUpTable lookupTable;

  private Timer shooterTimer;
  private boolean isTimerRunning;

  private Translation2d speaker;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Intake intake, Chassis chassis) {
    this.shooter = shooter;
    this.intake = intake;
    this.chassis = chassis;
    lookupTable = shooter.lookUpTable;
    state = shooter.shooterState;

    shooterTimer = new Timer();
    isTimerRunning = false;
    isReady = false;
    isfinished = false;
    
    upMotorVelocity = 0;
    downMotorVelocity = 0;
    testingUpMotorVelocity = 0;
    testingDownMotorVelocity = 0;
    distence = 0;
    distenceX = 0;

    speaker = RobotContainer.isRed ? Field.RedSpeaker : Field.Speaker;

    SmartDashboard.putData(this);
    addRequirements(shooter, intake);
    // Use addRequirements() here to declare subsystem dependencies.ll
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("testingUpMotorVelocity", this::getUpMotorVelocity, this::setUpMotorVelocity);
    builder.addDoubleProperty("testingDownMotorVelocity", this::getDownMotorVelocity, this::setDownMotorVelocity);
  }

  public double getUpMotorVelocity() {
    return this.testingUpMotorVelocity;
  }

  public void setUpMotorVelocity(double testingUpMotorVelocity) {
    this.testingUpMotorVelocity = testingUpMotorVelocity;
  }

  public double getDownMotorVelocity() {
    return this.testingDownMotorVelocity;
  }

  public void setDownMotorVelocity(double testingDownMotorVelocity) {
    this.testingDownMotorVelocity = testingDownMotorVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isTimerRunning = false;
    shooterTimer.stop();
    shooterTimer.reset();
    state = shooter.shooterState;
    speaker = RobotContainer.isRed ? Field.RedSpeaker : Field.Speaker;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // distenceX = chassis.getPose().getX(); 
    // if (distenceX >= DISTANCES.WING_DISTANCE && distenceX < DISTANCES.RIVAL_WING_DISTANCE && state != STATE.AMP
    //     && state != STATE.STAGE && state != STATE.SUBWOFFER && state != STATE.TESTING) {
    //   state = STATE.DELIVERY_MID;
    // }
    // if (distenceX >= DISTANCES.RIVAL_WING_DISTANCE && state != STATE.AMP && state != STATE.STAGE
    //     && state != STATE.SUBWOFFER && state != STATE.TESTING) {
    //   state = STATE.DELIVERY_RIVAL;
    // }

    switch (state) {
      case AMP:
        upMotorVelocity = AMP_VAR.MOTOR_UP_AMP_VELOCITY;
        downMotorVelocity = AMP_VAR.MOTOR_DOWN_AMP_VELOCITY;
        break;

      case STAGE:
        upMotorVelocity = STAGE_VAR.MOTOR_UP_STAGE_VELOCITY;
        downMotorVelocity = STAGE_VAR.MOTOR_DOWN_STAGE_VELOCITY;
        break;

      case SUBWOFFER:
        upMotorVelocity = SUBWOFFER_VAR.MOTOR_UP_SUBWOFFER_VELOCITY;
        downMotorVelocity = SUBWOFFER_VAR.MOTOR_DOWN_SUBWOFFER_VELOCITY;
        break;

      case SPEAKER:
        distence = speaker.minus(chassis.getPose().getTranslation()).getNorm();
        double[] lookUpTableData = lookupTable.get(distence);
        upMotorVelocity = lookUpTableData[1];
        downMotorVelocity = lookUpTableData[2];
        break;

      case TESTING:
        upMotorVelocity = testingUpMotorVelocity;
        downMotorVelocity = testingDownMotorVelocity;
        break;

      case DELIVERY:
        upMotorVelocity = DELIVERY_VAR.MOTOR_UP_DELIVERY_VEL;
        downMotorVelocity = DELIVERY_VAR.MOTOR_DOWN_DELIVERY_VEL;
        break;

      case IDLE:
        upMotorVelocity = 0;
        downMotorVelocity = 0;
        break;
    }

    shooter.isShotoerReady = Ready.isUpMotorReady(upMotorVelocity) && Ready.isDownMotorReady(downMotorVelocity);

    shooter.pidMotorVelocity(upMotorVelocity, downMotorVelocity);

    isReady = isDriverOverwriteShooter;
    if (isReady) {
      shooter.setFeedingPower(SHOOTER_POW.FEEDING_MOTOR_POWER);
      intake.setPowerToMotors(SHOOTER_POW.INTAKE_MOTOR_POWER);
      // intake.motorPickUpSetPower(0.5);

      System.out.println(shooterTimer.get());
      if (!isTimerRunning) {
        shooterTimer.start();
        isTimerRunning = true;
      }

      if (shooterTimer.get()*1000 >= SHOOTER_ATRIBUTES.MIL_SEC_TO_SHOOT) {
        shooterTimer.stop();
        shooterTimer.reset();
        isTimerRunning = false;
        isfinished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (state != STATE.TESTING && state != STATE.IDLE && state != STATE.DELIVERY) {
      angleChanging.angleState = STATE.SPEAKER;
      shooter.shooterState = STATE.SPEAKER;
    }

    shooter.setMotorPower(0, 0);
    shooter.setFeedingPower(0);
    intake.setPowerToMotors(0);
    shooterTimer.stop();
    shooterTimer.reset();
    isTimerRunning = false;
    isfinished = false;
    isReady = false;
    isDriverOverwriteShooter = false;
    shooter.isShotoerReady = false;

    if (!interrupted) {
      intake.isNoteInIntake = false;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfinished || state == STATE.IDLE;
  }
}
