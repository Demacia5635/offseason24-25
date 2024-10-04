package frc.robot.Intake.Subsystem;
import static frc.robot.Intake.IntakeConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Intake.IntakeConstants.NotePosition;
public class Intake extends SubsystemBase {
  private TalonFX motorPickUp;//floor to intake
  private TalonFX motorMove;// intake to shooter
  public AnalogInput analogInput;
  TalonFXConfiguration configBase;

  public static NotePosition currentPosition;
  public static boolean isNoteInIntake = false;

  public static double testFirstTouch;
  public static double testSecondTouch;
  public static double testIrSensor;
  public static double testNoNote;

  

  public Intake() {
    motorMove = new TalonFX(INTAKE_MOTOR_UP_ID,CANBUS);
    motorPickUp = new TalonFX(INTAKE_MOTOR_DOWN_ID,CANBUS);
    configBase = new TalonFXConfiguration();

    configBase.MotorOutput.Inverted = IS_IVERTED_MOTOR_MOVE
    ? InvertedValue.CounterClockwise_Positive
    : InvertedValue.Clockwise_Positive;

    configBase.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorMove.getConfigurator().apply(configBase);

    configBase.MotorOutput.Inverted = IS_IVERTED_MOTOR_PICKUP
    ? InvertedValue.CounterClockwise_Positive
    : InvertedValue.Clockwise_Positive;
    motorMove.getConfigurator().apply(configBase);
  }

  @Override
  public void periodic() {}

  /** sets power to motor Move */
  public void motorMoveSetPower(double power){
    motorMove.set(power);
  }
  /** sets power to motor Pick Up */
    public void motorPickUpSetPower(double power){
    motorPickUp.set(power);
  }

  /** sets Power to both motors  */
  public void setPowerToMotors(double power){
    motorPickUp.set(power);
    motorMove.set(power);
  }
  /** give me information when the sensor is read by the note */
  public boolean isNote(){// It is for IR sensor DONT DELETE
    return analogInput.getVoltage() > NOTE_VOLTEGE;
  }

  /**checks if the amper is heigh than x(for now i dont know how much and cant test yet) current it returns true on motorMove!! */
  public boolean AmperHighMotorMove(){
    return motorMove.getSupplyCurrent().getValue() >= NOTE_CURRENT;
  }

    /**checks if the amper is heigh than x(for now i dont know how much and cant test yet) current it returns true on PickUp!! */
    public boolean AmperHighMotorPickUp(){
    return motorPickUp.getSupplyCurrent().getValue() >= NOTE_CURRENT;
  }


  public void setMotorsCoast(){
    configBase.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorMove.getConfigurator().apply(configBase);
    motorPickUp.getConfigurator().apply(configBase);

  }

  public void setMotorsBrake(){
    configBase.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorMove.getConfigurator().apply(configBase);
    motorPickUp.getConfigurator().apply(configBase);
  }


  public void setTestFirstTouch(double power){
    testFirstTouch = power;
  }
  public double getTestFirstTouch(){
    return testFirstTouch;
  }


  public void setTestSecondTouch(double power){
    testSecondTouch = power;
  }
  public double getTestSecondTouch(){
    return testSecondTouch;
  }

  public void setTestNoNote(double power){
    testNoNote = power;
  }
  public double getTestNoNote(){
    return testNoNote;
  }
  public double getCurrentOfMotorMove(){
    return motorMove.getSupplyCurrent().getValue();
  }

  public double getCurrentOfPickUp(){
    return motorPickUp.getSupplyCurrent().getValue();
  }
  public String getcurrentPosition(){
    return currentPosition.toString();
  }


  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("testFirstTouch",this::getTestFirstTouch,this::setTestFirstTouch);
    builder.addDoubleProperty("testSecondTouch",this::getTestSecondTouch,this::setTestSecondTouch);
    builder.addDoubleProperty("testNoNote", this::getTestNoNote, this::setTestNoNote);
    builder.addDoubleProperty("current of motor Move",this::getCurrentOfMotorMove,null);
    builder.addDoubleProperty("current of motor PickUp",this::getCurrentOfPickUp,null);
    builder.addStringProperty("currentPosition",this::getcurrentPosition,null);
  }
}
