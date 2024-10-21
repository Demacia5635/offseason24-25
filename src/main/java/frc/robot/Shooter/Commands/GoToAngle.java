// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.Shooter.ShooterConstants.AMP_VAR;
import frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_VAR;
import frc.robot.Shooter.ShooterConstants.STAGE_VAR;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.ShooterConstants.SUBWOFFER_VAR;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.utils.LookUpTable;
import frc.robot.Shooter.utils.Ready;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;

import static frc.robot.Shooter.ShooterConstants.*;

public class GoToAngle extends Command {
  /** Creates a new setShooting. */
  private LookUpTable lookupTable;
  private AngleChanger angleChanger;
  private Chassis chassis;

  private double wantedAngle;
  private double testingAngle;
  private double distence;
  private double distenceX;

  public STATE state;
  public static boolean isAngleReady = false;

  private Translation2d speaker;

  public GoToAngle(AngleChanger angleChanger, Chassis chassis) {
    this.angleChanger = angleChanger;
    this.chassis = chassis;
    state = angleChanger.angleState;
    lookupTable = angleChanger.lookUpTable;

    wantedAngle = angleChanger.getAngle();
    testingAngle = angleChanger.getAngle();
    distence = 0;
    distenceX = 0;

    speaker = RobotContainer.isRed ? Field.RedSpeaker : Field.Speaker;

    SmartDashboard.putData(this);
    addRequirements(angleChanger);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Angle", this::getWantedAngle, this::setWantedAngle);

    LogManager.addEntry("Shooter/AngleChanging/Wanted angle", ()-> wantedAngle);
    LogManager.addEntry("Shooter/AngleChanging/distence from speaker", ()-> speaker.minus(chassis.getPose().getTranslation()).getNorm());
  }

  public double getWantedAngle() {
    return this.testingAngle;
  }

  public void setWantedAngle(double testingAngle) {
    this.testingAngle = testingAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = angleChanger.angleState;
    speaker = RobotContainer.isRed ? Field.RedSpeaker : Field.Speaker;
  }

  // Called every time the scheduler runs while the command is scheduled.

  /**
   * index: 0 - distance, 1 - angle, 2 - up vel, 3 - down vel
   * 
   * @param lookUpTableData index of angle - 0, up vel - 1, down vel - 2
   * 
   */
  @Override
  public void execute() {
    state = angleChanger.angleState;
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
        wantedAngle = AMP_VAR.AMP_ANGLE;
        break;

      case STAGE:
        wantedAngle = STAGE_VAR.STAGE_ANGLE;

        break;

      case SUBWOFFER:
        wantedAngle = SUBWOFFER_VAR.SUBWOFFER_ANGLE;
        break;

      case SPEAKER:
        distence = speaker.minus(chassis.getPose().getTranslation()).getNorm()+0.35;
        double[] speakerLookUpTableData = lookupTable.get(distence);
        wantedAngle = speakerLookUpTableData[0];
        break;

      case DELIVERY:
        wantedAngle = DELIVERY_VAR.DELIVERY_ANGLE;
      
      case TESTING:
        wantedAngle = testingAngle;
        break;

      case IDLE:
        wantedAngle = angleChanger.getAngle();
        break;
    }

    isAngleReady = Ready.isAngleReady(wantedAngle);

    // angleChanger.goToAngle(wantedAngle);
    angleChanger.goToAnglePositionVol(wantedAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleChanger.setMotorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
