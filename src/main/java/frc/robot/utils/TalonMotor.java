package frc.robot.utils;


import static frc.robot.chassis.ChassisConstants.CYCLE_DT;
import static frc.robot.chassis.ChassisConstants.MAX_STEER_VELOCITY;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.chassis.ChassisConstants;

/** Add your docs here. */
public class TalonMotor extends TalonFX {
  TalonConfig config;
  String name;
  TalonFXConfiguration cfg;
  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  VoltageOut voltageOut = new VoltageOut(0);

  DutyCycleOut dutyCycle = new DutyCycleOut(0);
  // MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
  LogManager.LogEntry dutyCycleEntry;
  LogManager.LogEntry velocityEntry;
  LogManager.LogEntry positionEntry;

  /*TODO test limits */
  boolean isLimitsEnable;
  double maxVel;
  double maxAcc;
  double maxDeltaVel;


  public TalonMotor(TalonConfig config) {
    super(config.id, config.canbus);
    this.config = config;
    name = config.name;
    configMotor();
    addLog();
    this.maxDeltaVel = maxAcc * CYCLE_DT;
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
    //motionMagicVoltage.UpdateFreqHz = 200;

    positionVoltage.UpdateFreqHz = 200;
  
    
    

    getConfigurator().apply(cfg);
    getPosition().setUpdateFrequency(200);
    getVelocity().setUpdateFrequency(200);
    getAcceleration().setUpdateFrequency(200);
    getMotorVoltage().setUpdateFrequency(200);

    isLimitsEnable = config.isLimitsEnable;
    maxVel = config.maxVel;
    maxAcc = config.maxAccel;

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
   * set power from -12-12 no PID/FF
   */
  public void setVoltage(double voltage) {
    setControl(voltageOut.withOutput(voltage));
  }
  /**
   * gets rotations per secon
   * set volocity to motor with PID and FF
   */
  public void setVelocity(double velocity) {
    setControl(velocityVoltage.withVelocity(velocity));
    velocityEntry.log(velocity);
  }
  /**
   * gets R/s
   * set volocity to motor with PID and FF
   */
  public void setVelocityRPS(double velocity) {
    setVelocity(velocity);
  }
  /**
   * gets m/s
   * set volocity to motor with PID and FF
   */
  public void setVelocityMPS(double velocity, double radius) {
    setVelocity(Utils.mpsToRps(velocity, radius ));
  }
  private double steerOptimization(double currentPosition, double wantedPosition) {
    // Normalize angles to the range [0, 1)
    
    currentPosition = currentPosition % 1.0;
    wantedPosition = wantedPosition % 1.0;
    

    // Calculate the difference
    double rotationsDiff = wantedPosition - currentPosition;

    // Normalize the difference to the range [-0.5, 0.5)
    if (rotationsDiff > 0.5) {
        rotationsDiff -= 1.0;
    } else if (rotationsDiff < -0.5) {
        rotationsDiff += 1.0;
    }

    return rotationsDiff; // Return the shortest path to move
  }

  /**
  * only use on swerve module steer
  * @param position the wanted poistion 
  * @param maxError the max vaiable error
  * @param isWithAccel is testing for max accelaration (NEED TO TEST)
  */
  public void setMotorPositionOptimized(Rotation2d position, Rotation2d maxError, boolean isWithAccel){
    double wantedPosition = steerOptimization(getCurrentPosition().getRotations(), position.getRotations()) + getCurrentPosition().getRotations();
    if(Math.abs(wantedPosition - getCurrentPosition().getRotations()) <= maxError.getRotations()) set(0);  
    else{
        if(getCurrentVelocity().getRotations() + maxDeltaVel < positionVoltage.withPosition(wantedPosition).Velocity 
          && isWithAccel){
          setVelocity(getCurrentVelocity().getRotations() + maxDeltaVel);
        }
        else if(positionVoltage.withPosition(wantedPosition).Velocity >= maxVel) {
          setVelocity(ChassisConstants.MAX_STEER_VELOCITY);
        } else {
          setControl(positionVoltage.withPosition(wantedPosition));
        }
      }
    
    positionEntry.log(position.getRotations());
  } 

  


  


  /**
   * set position to drive to in rotations
   */
  public void setMotorPosition(Rotation2d position, Rotation2d maxError) {
    
    double currentPositionRounded = Math.round(getCurrentPosition().getRotations());
    double wantedPosition = currentPositionRounded + position.getRotations();
   
    if(Math.abs(wantedPosition - getCurrentPosition().getRotations()) <= maxError.getRotations()) set(0);
    else setControl(positionVoltage.withPosition(wantedPosition).withSlot(0));
    positionEntry.log(position.getRotations());

  }



  /**
   * get position in rotations
   */
  public Rotation2d getCurrentPosition() {
    return Rotation2d.fromRotations(getPosition().getValueAsDouble());
  }



  /**
   * get position in degrees
   */
  public double getCurrentPositionAsDegrees() {
    return getCurrentPosition().getDegrees();
  }
  /**
   * get Velosity in rotations per seconds
   */
  public Rotation2d getCurrentVelocity() {
    return Rotation2d.fromRotations(getVelocity().getValueAsDouble());
  }
  /**
   * get Velosity in meters per second
   */
  public double getCurrentVelocityInMS(double RadiusInM) {
    return Utils.rpsToMps(getCurrentVelocity().getRotations(), RadiusInM);
  }
  /**
   * get Velosity in degrees per seconds
   */
  public double getCurrentVelocityDegrees(){
    return getCurrentVelocity().getDegrees();
  }


  public String name() {
    return name;
  }
  

}