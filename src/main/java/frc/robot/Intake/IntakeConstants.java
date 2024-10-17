package frc.robot.Intake;

import frc.robot.Constants;

/**
 * The intake constants
 */
public class IntakeConstants {

  /**
   * The id of motors and the name of the canivore
   */
  public static class IdOfMotors{
    public static final int MOTOR_PICKUP_ID = 40;
    public static final int MOTOR_MOVE_ID = 42;
    public static final int IR_SENSOR = 3;

    public static final String CANBUS = "rio"; // Constants.CANBUS;
  }

  /**
   * The starting configurations of the motors
   */
  public static class Config{
    public static final Boolean IS_BRAKE_MOTORS = true;
    public static final Boolean IS_INVERTED_MOTOR_MOVE = true;
    public static final Boolean IS_INVERTED_MOTOR_PICKUP = false;
    public static final Boolean IS_TESTING = true;
  }

  /**The voltege of the sensor when he detects the note */
  public static final double NOTE_VOLTEGE = 3.0;

  //The amper of when the not is fed to the intake
  public static final double NOTE_AMPER = 25.0;

  /**The amount of time the intake command will auto stop (in miliseconds) */
  public static final double STOP_COMMAND_TIME = 20000;

  /**The amount of time the intake command will stop after takinng note */
  public static final double STOP_AFTER_NOTE = 500;

  /**
   * The current position of the note to power fo the motors
   */
  public enum NotePosition{
    NO_NOTE(1),
    FIRST_TOUCH(1);

    /** The voltage to the motors */
    public final double power;

    /**
     * Constractor for enum
     * @param power The voltage to the motors thst you get
     */
    NotePosition(double power) {
      this.power = power;
    }
  }
}