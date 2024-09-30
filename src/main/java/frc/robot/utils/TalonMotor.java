package frc.robot.utils;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class TalonMotor extends TalonFX {
  TalonConfig config;
  String name;
  TalonFXConfiguration cfg;
  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

  DutyCycleOut dutyCycle = new DutyCycleOut(0);
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  LogManager.LogEntry dutyCycleEntry;
  LogManager.LogEntry velocityEntry;
  LogManager.LogEntry positionEntry;


  public TalonMotor(TalonConfig config) {
    super(config.id, config.canbus);
    this.config = config;
    name = config.name;
    configMotor();
    addLog();
    LogManager.log(name + " motor initialized");
  }

  private void configMotor() {
    cfg = new TalonFXConfiguration();
    cfg.CurrentLimits.SupplyCurrentLimit = config.maxCurrent;
    cfg.CurrentLimits.SupplyCurrentThreshold = config.maxCurrentTriggerTime;
    cfg.CurrentLimits.SupplyTimeThreshold = config.maxCurrentTriggerTime;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.rampUpTime;
    cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.rampUpTime;

    cfg.MotorOutput.Inverted = config.inverted ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    cfg.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    cfg.MotorOutput.PeakForwardDutyCycle = config.maxVolt / 12.0;
    cfg.MotorOutput.PeakReverseDutyCycle = config.minVolt / 12.0;

    cfg.Slot0.kP = config.pid.kp;
    cfg.Slot0.kI = config.pid.ki;
    cfg.Slot0.kD = config.pid.kd;
    cfg.Slot0.kS = config.pid.ks; 
    cfg.Slot0.kV = config.pid.kv;
    cfg.Slot0.kA = config.pid.ka;
    cfg.Slot0.kG = config.pid.kg;
    if(config.pid1 != null) {
      cfg.Slot1.kP = config.pid1.kp;
      cfg.Slot1.kI = config.pid1.ki;
      cfg.Slot1.kD = config.pid1.kd;
      cfg.Slot1.kS = config.pid1.ks; 
      cfg.Slot1.kV = config.pid1.kv;
      cfg.Slot1.kA = config.pid1.ka;
      cfg.Slot1.kG = config.pid1.kg;
    }
    if(config.pid2 != null) {
      cfg.Slot2.kP = config.pid2.kp;
      cfg.Slot2.kI = config.pid2.ki;
      cfg.Slot2.kD = config.pid2.kd;
      cfg.Slot2.kS = config.pid2.ks; 
      cfg.Slot2.kV = config.pid2.kv;
      cfg.Slot2.kA = config.pid2.ka;
      cfg.Slot2.kG = config.pid2.kg;
    }

    cfg.Voltage.PeakForwardVoltage = config.maxVolt;
    cfg.Voltage.PeakReverseVoltage = config.minVolt;

    cfg.Feedback.SensorToMechanismRatio = config.motorRatio;
    cfg.MotionMagic.MotionMagicAcceleration = config.motionMagicAccel;
    cfg.MotionMagic.MotionMagicCruiseVelocity = config.motionMagicVelocity;
    cfg.MotionMagic.MotionMagicJerk = config.motionMagicJerk;
    cfg.MotionMagic.MotionMagicExpo_kA = config.pid.ka;
    cfg.MotionMagic.MotionMagicExpo_kV = config.pid.kv;

    velocityVoltage.UpdateFreqHz = 200;
    dutyCycle.UpdateFreqHz = 200;
    motionMagicVoltage.UpdateFreqHz = 200;
    

    getConfigurator().apply(cfg);
    getPosition().setUpdateFrequency(200);
    getVelocity().setUpdateFrequency(200);
    getAcceleration().setUpdateFrequency(200);
    getMotorVoltage().setUpdateFrequency(200);

  }
  /*
   * set motor to brake or coast
   */
  public void setBrake(boolean brake) {
    this.getConfigurator().refresh(cfg.MotorOutput);
    cfg.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    getConfigurator().apply(cfg.MotorOutput);
  }

  private void addLog() {
    LogManager.addEntry(name + "/position", this::getPosition);// rotation
    LogManager.addEntry(name + "/position in degres", this::getCurrentPositionAsDegrees);// degrees
    LogManager.addEntry(name + "/Velocity", this::getVelocity);// rotation per seconds
    LogManager.addEntry(name + "/Acceleration", this::getAcceleration);// rotation per seconds^2
    LogManager.addEntry(name + "/Voltage", this::getMotorVoltage);
    LogManager.addEntry(name + "/Current", this::getStatorCurrent);
    LogManager.addEntry(name + "/CloseLoopError", this::getClosedLoopError);
    LogManager.addEntry(name + "/CloseLoopOutput", this::getClosedLoopOutput);
    LogManager.addEntry(name + "/CloseLoopP", this::getClosedLoopProportionalOutput);
    LogManager.addEntry(name + "/CloseLoopI", this::getClosedLoopIntegratedOutput);
    LogManager.addEntry(name + "/CloseLoopD", this::getClosedLoopDerivativeOutput);
    LogManager.addEntry(name + "/CloseLoopFF", this::getClosedLoopFeedForward);
    LogManager.addEntry(name + "/CloseLoopSP", this::getClosedLoopReference);
    dutyCycleEntry = LogManager.getEntry(name + "/setDutyCycle");
    velocityEntry = LogManager.getEntry(name + "/setVelocity");
    positionEntry = LogManager.getEntry(name + "/setPosition");
  }
  /**
   * set power from 1 to -1 (v/12) no PID/FF
   */
  public void setDuty(double power) {
    setControl(dutyCycle.withOutput(power));
    dutyCycleEntry.log(power);
  }
  /**
   * gets rotations per secon
   * set volocity to motor with PID and FF
   */
  public void setVelocity(double velocity, double feedForward) {
    setControl(velocityVoltage.withVelocity(velocity).withFeedForward(feedForward));
    velocityEntry.log(velocity);
  }
  /**
   * gets R/s
   * set volocity to motor with PID and FF
   */
  public void setVelocityRPS(double velocity) {
    setVelocity(velocity,0);
  }
  /**
   * gets m/s
   * set volocity to motor with PID and FF
   */
  public void setVelocityMPS(double velocity, double radius) {
    setVelocity(Utils.mpsToRps(velocity, radius ), 0);
  }

  /**
   * set position to drive to in rotations
   */
  public void setMotorPosition(double position/*in rotation */, double feedForward) {
    // System.out.println("==================");
    // System.out.println(name + " current pos: " + getCurrentPosition());
    // System.out.println(name + " target pos: " + position);
    // System.out.println(name + " current vel: " + getCurrentVelocity());
    // System.out.println("==================");
    //setControl(positionEntry)
    setControl(motionMagicVoltage.withPosition(position));
    positionEntry.log(position);
  }
  /**
   * set position to drive to in rotations
   */
  public void setMotorPosition(double position/*in rotation */) {
    setMotorPosition(position, 0);
  }


  /**
   * get position in rotations
   */
  public double getCurrentPosition() {
    return getPosition().getValueAsDouble();
  }
  /**
   * get position in Rotation2D
   */
  public Rotation2d getCurrentPositionAsRotation2d() {
    return Rotation2d.fromRotations(getCurrentPosition());
  }
  /**
   * get position in degrees
   */
  public double getCurrentPositionAsDegrees() {
    return getCurrentPositionAsRotation2d().getDegrees();
  }
  /**
   * get Velosity in rotations per seconds
   */
  public double getCurrentVelocity() {
    return getVelocity().getValueAsDouble();
  }
  /**
   * get Velosity in meters per second
   */
  public double getCurrentVelocityInMPS(double RadiusInM) {
    return Utils.rpsToMps(getCurrentVelocity(), RadiusInM);
  }
  /**
   * get Velosity in degrees per seconds
   */
  public double getCurrentVelocityDegrees(){
    return Rotation2d.fromRotations(getCurrentVelocity()).getDegrees();
  }


  public String name() {
    return name;
  }
  

}