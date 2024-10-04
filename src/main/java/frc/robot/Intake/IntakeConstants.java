// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import java.util.Collections;
import java.util.HashMap;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Intake.Subsystem.Intake;

/** Add your docs here. */
public class IntakeConstants {
  public static final int INTAKE_MOTOR_DOWN_ID = -1;
  public static final int INTAKE_MOTOR_UP_ID = -1;
  public static final String CANBUS = "canivore";
  public static final double NOTE_VOLTEGE = 4.0;
  public static final double NOTE_CURRENT = 0.0;
  public static final Boolean IS_BRAKE_MOTORS = true;
  public static final Boolean IS_IVERTED_MOTOR_MOVE = true;
  public static final Boolean IS_IVERTED_MOTOR_PICKUP = true;
  public static final Boolean IS_TESTING = true;
  public enum NotePosition{
    NO_NOTE(1),
    FIRST_TOUCH(0.6),
    IR_SENSOR(0.3),
    SECOND_TOUCH(0.25),
    FULLY_IN(0.0),

    TEST_NO_NOTE(Intake.testNoNote),
    TEST_FIRST_TOUCH(Intake.testFirstTouch),
    TEST_SECOND_TOUCH(Intake.testSecondTouch),
    TEST_IR_SENSOR(Intake.testIrSensor);

    public double power;
    /**
     * constractor for en
     * @param power
     */
    NotePosition(double power) {
      this.power = power;
    }
  }
}