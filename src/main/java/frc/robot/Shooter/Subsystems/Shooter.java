// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;

import static frc.robot.Shooter.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.utils.LookUpTable;
import frc.robot.utils.LogManager;

public class Shooter extends SubsystemBase {
  
  private TalonFX motorUp;
  private TalonFX motorDown;
  private TalonSRX motorFeeding;

  private TalonFXConfiguration configUp;
  private TalonFXConfiguration configDown;

  private DutyCycleOut m_request;
  private VelocityVoltage velocityVoltage;
  public STATE shooterState;
  
  public LookUpTable lookUpTable;
  
  /** Creates a new Shooter. */
  public Shooter() {
    shooterState = STATE.SPEAKER;

    motorFeeding = new TalonSRX(MOTOR_IDS.MOTOR_FEEDING_ID);
    motorDown = new TalonFX(MOTOR_IDS.MOTOR_DOWN_ID, MOTOR_IDS.CANBUS);
    motorUp = new TalonFX(MOTOR_IDS.MOTOR_UP_ID, MOTOR_IDS.CANBUS);

    m_request = new DutyCycleOut(0.0).withUpdateFreqHz(SHOOTER_CONFIGS.FREQHZ);
    velocityVoltage = new VelocityVoltage(0).withSlot(0).withUpdateFreqHz(SHOOTER_CONFIGS.FREQHZ);

    configUp = new TalonFXConfiguration();
    configDown = new TalonFXConfiguration();
    
    configUp.MotorOutput.NeutralMode = SHOOTER_CONFIGS.IS_SHOOTING_MOTORS_BRAKE
    ? NeutralModeValue.Brake
    : NeutralModeValue.Coast;

    configDown.MotorOutput.NeutralMode = SHOOTER_CONFIGS.IS_SHOOTING_MOTORS_BRAKE
    ? NeutralModeValue.Brake
    : NeutralModeValue.Coast;

    configUp.Slot0.kP = SHOOTER_PID_FF.UP_MOTOR_KP;
    configUp.Slot0.kI = SHOOTER_PID_FF.UP_MOTOR_KI;
    configUp.Slot0.kD = SHOOTER_PID_FF.UP_MOTOR_KD;
    configUp.Slot0.kS = SHOOTER_PID_FF.UP_MOTOR_KS;
    configUp.Slot0.kV = SHOOTER_PID_FF.UP_MOTOR_KV;

    configDown.Slot0.kP = SHOOTER_PID_FF.DOWN_MOTOR_KA;
    configDown.Slot0.kI = SHOOTER_PID_FF.DOWN_MOTOR_KI;
    configDown.Slot0.kD = SHOOTER_PID_FF.DOWN_MOTOR_KD;
    configDown.Slot0.kS = SHOOTER_PID_FF.DOWN_MOTOR_KS;
    configDown.Slot0.kV = SHOOTER_PID_FF.DOWN_MOTOR_KV;

    configUp.MotorOutput.Inverted = SHOOTER_CONFIGS.IS_UP_MOTOR_INVERT ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    configDown.MotorOutput.Inverted = SHOOTER_CONFIGS.IS_DOWN_MOTOR_INVERT ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    motorUp.getConfigurator().apply(configUp);
    motorDown.getConfigurator().apply(configDown);
    
    motorFeeding.configFactoryDefault();
    motorFeeding.setInverted(SHOOTER_CONFIGS.IS_FEEDING_MOTOR_INVERT);
    motorFeeding.setNeutralMode(SHOOTER_CONFIGS.IS_FEEDING_MOTOR_BRAKE ? NeutralMode.Brake : NeutralMode.Coast);

    lookUpTable = new LookUpTable(LOOKUP_TABLE_DATA.DATA);

    SmartDashboard.putData("Shooter", this);
  }

  public void setMotorPower(double upPower, double downPower){
    motorUp.setControl(m_request.withOutput(upPower));
    motorDown.setControl(m_request.withOutput(downPower));
  }
  


  public void setFeedingPower(double power){
    motorFeeding.set(ControlMode.PercentOutput, power);
  }

  public double getDownMotorVel(){
    return motorDown.getVelocity().getValueAsDouble();
  }

  public double getUpMotorVel(){
    return motorUp.getVelocity().getValueAsDouble();
  }

  public void pidMotorVelocity(double upVel, double downVel){
    motorUp.setControl(velocityVoltage.withVelocity(upVel)); // .withFeedForward(ShooterUtils.getUpMotorFF(getUpMotorVel())));
    motorDown.setControl(velocityVoltage.withVelocity(downVel)); // .withFeedForward(ShooterUtils.getDownMotorFF(getDownMotorVel())));
  }

  public void setShootingNeutralMode(boolean isBrake){
    configUp.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    configDown.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    motorUp.getConfigurator().apply(configUp);
    motorDown.getConfigurator().apply(configDown);
  }

  public void setFeedingNeutralMode(boolean isBrake){
    motorFeeding.setNeutralMode(isBrake ? NeutralMode.Brake : NeutralMode.Coast);
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);;

    SmartDashboard.putData("change shooting brake", new InstantCommand(()-> 
      setShootingNeutralMode(configUp.MotorOutput.NeutralMode==NeutralModeValue.Brake ? false : true)
       , this).ignoringDisable(true));

    SmartDashboard.putData("change feeding brake", new InstantCommand(()-> 
      setFeedingNeutralMode(configUp.MotorOutput.NeutralMode==NeutralModeValue.Brake ? false : true)
       , this).ignoringDisable(true));

    builder.addStringProperty("Shooter state", ()->shooterState.toString(), null);

    LogManager.addEntry("Shooter/UpMotor/Velocity", this::getUpMotorVel);
    LogManager.addEntry("Shooter/UpMotor/Acceleration", ()-> motorUp.getAcceleration().getValueAsDouble());
    LogManager.addEntry("Shooter/UpMotor/Voltage", ()-> motorUp.getMotorVoltage().getValueAsDouble());
    LogManager.addEntry("Shooter/UpMotor/Current", ()-> motorUp.getSupplyCurrent().getValueAsDouble());
    LogManager.addEntry("Shooter/UpMotor/CloseLoopError", ()-> motorUp.getClosedLoopError().getValueAsDouble());
    LogManager.addEntry("Shooter/UpMotor/CloseLoopOutput", ()-> motorUp.getClosedLoopOutput().getValueAsDouble());
    LogManager.addEntry("Shooter/UpMotor/CloseLoopP", ()-> motorUp.getClosedLoopProportionalOutput().getValueAsDouble());
    LogManager.addEntry("Shooter/UpMotor/CloseLoopI", ()-> motorUp.getClosedLoopIntegratedOutput().getValueAsDouble());
    LogManager.addEntry("Shooter/UpMotor/CloseLoopD", ()-> motorUp.getClosedLoopDerivativeOutput().getValueAsDouble());
    LogManager.addEntry("Shooter/UpMotor/CloseLoopFF", ()-> motorUp.getClosedLoopFeedForward().getValueAsDouble());
    LogManager.addEntry("Shooter/UpMotor/CloseLoopSP", ()-> motorUp.getClosedLoopReference().getValueAsDouble());

    LogManager.addEntry("Shooter/DownMotor/Velocity", this::getDownMotorVel);
    LogManager.addEntry("Shooter/DownMotor/Acceleration", ()-> motorDown.getAcceleration().getValueAsDouble());
    LogManager.addEntry("Shooter/DownMotor/Voltage", ()-> motorDown.getMotorVoltage().getValueAsDouble());
    LogManager.addEntry("Shooter/DownMotor/Current", ()-> motorDown.getSupplyCurrent().getValueAsDouble());
    LogManager.addEntry("Shooter/DownMotor/CloseLoopError", ()-> motorDown.getClosedLoopError().getValueAsDouble());
    LogManager.addEntry("Shooter/DownMotor/CloseLoopOutput", ()-> motorDown.getClosedLoopOutput().getValueAsDouble());
    LogManager.addEntry("Shooter/DownMotor/CloseLoopP", ()-> motorDown.getClosedLoopProportionalOutput().getValueAsDouble());
    LogManager.addEntry("Shooter/DownMotor/CloseLoopI", ()-> motorDown.getClosedLoopIntegratedOutput().getValueAsDouble());
    LogManager.addEntry("Shooter/DownMotor/CloseLoopD", ()-> motorDown.getClosedLoopDerivativeOutput().getValueAsDouble());
    LogManager.addEntry("Shooter/DownMotor/CloseLoopFF", ()-> motorDown.getClosedLoopFeedForward().getValueAsDouble());
    LogManager.addEntry("Shooter/DownMotor/CloseLoopSP", ()-> motorDown.getClosedLoopReference().getValueAsDouble());
    
    LogManager.addEntry("Shooter/FeedingMotor/Velocity", ()-> motorFeeding.getSelectedSensorVelocity()*10);
    LogManager.addEntry("Shooter/FeedingMotor/Voltage", ()-> motorFeeding.getMotorOutputVoltage());
    LogManager.addEntry("Shooter/FeedingMotor/Current", ()-> motorFeeding.getSupplyCurrent());
  }
}