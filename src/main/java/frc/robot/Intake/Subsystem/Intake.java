package frc.robot.Intake.Subsystem;

import static frc.robot.Intake.IntakeConstants.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Intake.IntakeConstants.Config;
import frc.robot.Intake.IntakeConstants.IdOfMotors;
import frc.robot.Intake.IntakeConstants.NotePosition;

/** The intake susystem */
public class Intake extends SubsystemBase {

  /** Intake to shooter motor */
  private TalonFX motorMove;
  /**Floor to intake motor */
  private TalonFX motorPickUp;

  /** the IR sensor */
  public AnalogInput analogIRSenor;

  /** Default config */
  TalonFXConfiguration configBase;
  /** Config for the intake to shooter motor */
  TalonFXConfiguration configMove;
  /** Config for the floor to intake motor */
  TalonFXConfiguration configPickUp;

  /** The position of the note */
  public static NotePosition currentPosition;

  /** If the note is the intake */
  public static boolean isNoteInIntake = false;

  /** 
   * Configars the motors and and the configs
   */
  public Intake() {
    motorMove = new TalonFX(IdOfMotors.INTAKE_MOTOR_UP_ID,IdOfMotors.CANBUS);
    motorPickUp = new TalonFX(IdOfMotors.INTAKE_MOTOR_DOWN_ID,IdOfMotors.CANBUS);
    configBase = new TalonFXConfiguration();

    configBase.MotorOutput.NeutralMode = Config.IS_BRAKE_MOTORS ?NeutralModeValue.Brake :NeutralModeValue.Coast;

    configMove = configBase;
    configPickUp = configBase;

    configMove.MotorOutput.Inverted = Config.IS_INVERTED_MOTOR_MOVE
    ? InvertedValue.CounterClockwise_Positive
    : InvertedValue.Clockwise_Positive;

    configPickUp.MotorOutput.Inverted = Config.IS_INVERTED_MOTOR_PICKUP
    ? InvertedValue.CounterClockwise_Positive
    : InvertedValue.Clockwise_Positive;
    
    motorMove.getConfigurator().apply(configMove);
    motorPickUp.getConfigurator().apply(configPickUp);
  }

  /** Set power to motor Move */
  public void motorMoveSetPower(double power){
    motorMove.set(power);
  }
  /** Set power to motor Pick Up */
  public void motorPickUpSetPower(double power){
    motorPickUp.set(power);
  }

  /** Set power to both motors  */
  public void setPowerToMotors(double power){
    motorPickUp.set(power);
    motorMove.set(power);
  }

  /**
   * Gives me information when the sensor is read by the note
   * @return if the voltage that the sensor gave was high than NOTE_VOLTEGE
   */
  public boolean isNote(){
    return analogIRSenor.getVoltage() > NOTE_VOLTEGE;
  }

  
  /**
   * Checks if the note is in the move motor
   * @return is the amper is high for the motor move  
   */
  public boolean AmperHighMotorMove(){
    return motorMove.getSupplyCurrent().getValue() >= NOTE_AMPER;
  }



  /**
   * Checks if the note is in the pick up motor
   * @return is the amper is high for the motor pick up 
   */
  public boolean AmperHighMotorPickUp(){
    return motorPickUp.getSupplyCurrent().getValue() >= NOTE_AMPER;
  }

  /**
   * Changese the configurations of the motors to coast 
   */
  public void setMotorsCoast(){
    configBase.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configPickUp.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorMove.getConfigurator().apply(configBase);
    motorPickUp.getConfigurator().apply(configPickUp);

  }

  /**
   * Changese the configurations of the motors to brake 
   */
  public void setMotorsBrake(){
    configBase.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configPickUp.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorMove.getConfigurator().apply(configBase);
    motorPickUp.getConfigurator().apply(configPickUp);
  }

  /**
   * Gives me the amper of the motor move than i use it for the shuffle-board
   * @return the amper of motor move
   */
  public double getCurrentOfMotorMove(){
    return motorMove.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Gives me the amper of the motor pick up than i use it fo the shuffle-board
   * @return the amper of motor pick up
   */
  public double getCurrentOfPickUp(){
    return motorPickUp.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Gives me the name of the currentPosition and tha i use it in the shuffle-board
   * @return the name of the currentPosition
   */
  public String getcurrentPosition(){
    return currentPosition.toString();
  }

  /**
   * Gives me the voltage of the ir-sensor so i can use it in the shuffle-board
   * @return the voltage of the sensor
   */
  public double getVoltageSensor(){
    return analogIRSenor.getVoltage();
  }

  /**
   * The func that I send to the shuffle-board 
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Amper motor Move",this::getCurrentOfMotorMove,null);
    builder.addDoubleProperty("Amper of motor PickUp",this::getCurrentOfPickUp,null);
    builder.addStringProperty("Current Position",this::getcurrentPosition,null);
    builder.addDoubleProperty("voltage sensor",this::getVoltageSensor,null);
  }
}
