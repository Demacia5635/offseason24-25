// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Shooter.ShooterConstants.*;

import frc.robot.Shooter.utils.LookUpTable;
import frc.robot.Shooter.utils.ShooterUtils;
import frc.robot.utils.LogManager;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class AngleChanger extends SubsystemBase {

  private TalonFX angleChangingMotor;

  private TalonFXConfiguration config;
  private DutyCycleOut m_request;
  private VelocityVoltage velocityVoltage;
  private MotionMagicVoltage motionMagicVoltage;
  private PositionVoltage positionVoltage;

  public STATE angleState;

  public DigitalInput limitSwitch;

  public LookUpTable lookUpTable;

  private boolean isCalibrated;

  /** Creates a new AngleChanging. */
  public AngleChanger() {

    angleChangingMotor = new TalonFX(MOTOR_IDS.ANGLE_CHANGING_ID, MOTOR_IDS.CANBUS);
    config = new TalonFXConfiguration();
    angleState = STATE.IDLE;

    config.Slot0.kP = ANGLE_CHANGING_PID_FF.KP;
    config.Slot0.kI = ANGLE_CHANGING_PID_FF.KI;
    config.Slot0.kD = ANGLE_CHANGING_PID_FF.KD;
    config.Slot0.kS = ANGLE_CHANGING_PID_FF.KS;
    config.Slot0.kV = ANGLE_CHANGING_PID_FF.KV;
    config.Slot0.kA = ANGLE_CHANGING_PID_FF.KA;

    m_request = new DutyCycleOut(0.0).withUpdateFreqHz(ANGLE_CHANGING_CONFIGS.ANGLE_CHANGING_FREQHZ);
    velocityVoltage = new VelocityVoltage(0).withSlot(0).withUpdateFreqHz(ANGLE_CHANGING_CONFIGS.ANGLE_CHANGING_FREQHZ);
    motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0)
        .withUpdateFreqHz(ANGLE_CHANGING_CONFIGS.ANGLE_CHANGING_FREQHZ);
    positionVoltage = new PositionVoltage(0).withSlot(0).withUpdateFreqHz(ANGLE_CHANGING_CONFIGS.ANGLE_CHANGING_FREQHZ);

    // config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    // config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = -1;

    config.MotionMagic.MotionMagicCruiseVelocity = ANGLE_CHANGING_CONFIGS.ANGLE_CHANGING_MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = ANGLE_CHANGING_CONFIGS.ANGLE_CHANGING_MAX_Acceleration;
    config.MotionMagic.MotionMagicJerk = ANGLE_CHANGING_CONFIGS.ANGLE_CHANGING_MAX_JERK;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ANGLE_CHANGING_CONFIGS.ANGLE_VOLTAGE_RAMP;

    config.Feedback.SensorToMechanismRatio = ANGLE_CHANGING_CONFIGS.ANGLE_CHANGING_GEAR_RATIO;
    config.MotorOutput.Inverted = ANGLE_CHANGING_CONFIGS.IS_ANGLE_MOTOR_INVERT
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = ANGLE_CHANGING_CONFIGS.IS_ANGLE_MOTORS_BRAKE
        ? NeutralModeValue.Brake
        : NeutralModeValue.Coast;

    angleChangingMotor.getConfigurator().apply(config);

    lookUpTable = new LookUpTable(LOOKUP_TABLE_DATA.DATA);
    
    limitSwitch = new DigitalInput(MOTOR_IDS.LIMIT_SWITCH_ID);
    
    isCalibrated = false;

    SmartDashboard.putData(this);
  }

  public void setMotorPower(double power) {
    angleChangingMotor.setControl(m_request.withOutput(power));
  }

  public void goToAngle(double wantedAngle) {
    if (wantedAngle < ANGLE_CHANGING_VAR.MIN_ANGLE) {
      return ;
    }
    if (wantedAngle > ANGLE_CHANGING_VAR.TOP_ANGLE) {
      return ;
    }
    if (!isCalibrated) {
      return ;
    }

    double distance = ShooterUtils.angleToDistance(wantedAngle);
    angleChangingMotor.setControl(motionMagicVoltage.withPosition(distance));
  }

  public void angleChangingPID(double vel) {
    if (!isCalibrated) return ;
    angleChangingMotor.setControl(velocityVoltage.withVelocity(vel));
  }

  public void goToAnglePositionVol(double wantedAngle) {
    if (wantedAngle < ANGLE_CHANGING_VAR.MIN_ANGLE) {
      return ;
    }
    if (wantedAngle > ANGLE_CHANGING_VAR.TOP_ANGLE) {
      return ;
    }

    if (!isCalibrated) {
      return ;
    }

    double distance = ShooterUtils.angleToDistance(wantedAngle);
    angleChangingMotor.setControl(positionVoltage.withPosition(distance));
  }

  public double getAngleMotorVel() {
    return angleChangingMotor.getVelocity().getValue();
  }

  public void setBaseAngle() {
    angleChangingMotor.setPosition(ANGLE_CHANGING_VAR.BASE_DIS);
    isCalibrated = true;
  }

  public void setAngleNeutralMode(boolean isBrake) {
    config.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    angleChangingMotor.getConfigurator().apply(config);
  }

  public double getAngle() {
    return ShooterUtils.distanceToAngle(angleChangingMotor.getPosition().getValueAsDouble());
  }

  /**
   * 
   * @return if the limit switch is closed
   */
  // * @return returns if reverse hard limit is closed
  public boolean isMaxAngle() {
    return !limitSwitch.get();
    // return angleChangingMotor.getReverseLimit().getValue() ==
    // ReverseLimitValue.ClosedToGround;
  }

  @Override
  public void periodic() {
    if (isMaxAngle())
      setBaseAngle();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    SmartDashboard.putData("reset angle", new InstantCommand(() -> setBaseAngle(), this).ignoringDisable(true));
    SmartDashboard.putData("change shooting brake",
        new InstantCommand(
            () -> setAngleNeutralMode(config.MotorOutput.NeutralMode == NeutralModeValue.Brake ? false : true), this)
            .ignoringDisable(true));

    builder.addStringProperty("Angle changing state", ()-> angleState.toString(), null);

    LogManager.addEntry("Shooter/AngleChanging/Angle", this::getAngle);
    LogManager.addEntry("Shooter/AngleChanging/Distance", ()-> angleChangingMotor.getPosition().getValueAsDouble());
    LogManager.addEntry("Shooter/AngleChanging/LimitSwitch", ()-> isMaxAngle() ? 1 : 0);
    LogManager.addEntry("Shooter/AngleChanging/isCalibrated", ()-> isCalibrated ? 1 : 0);

    LogManager.addEntry("Shooter/AngleChanging/Velocity", this::getAngleMotorVel);
    LogManager.addEntry("Shooter/AngleChanging/Acceleration", () -> angleChangingMotor.getAcceleration().getValueAsDouble());
    LogManager.addEntry("Shooter/AngleChanging/Voltage", () -> angleChangingMotor.getMotorVoltage().getValueAsDouble());
    // LogManager.addEntry("Shooter/AngleChanging/Current", () -> angleChangingMotor.getSupplyCurrent().getValueAsDouble());
    // LogManager.addEntry("Shooter/AngleChanging/CloseLoopError", () -> angleChangingMotor.getClosedLoopError().getValueAsDouble());
    // LogManager.addEntry("Shooter/AngleChanging/CloseLoopOutput", () -> angleChangingMotor.getClosedLoopOutput().getValueAsDouble());
    // LogManager.addEntry("Shooter/AngleChanging/CloseLoopP", () -> angleChangingMotor.getClosedLoopProportionalOutput().getValueAsDouble());
    // LogManager.addEntry("Shooter/AngleChanging/CloseLoopI", () -> angleChangingMotor.getClosedLoopIntegratedOutput().getValueAsDouble());
    // LogManager.addEntry("Shooter/AngleChanging/CloseLoopD", () -> angleChangingMotor.getClosedLoopDerivativeOutput().getValueAsDouble());
    // LogManager.addEntry("Shooter/AngleChanging/CloseLoopFF", () -> angleChangingMotor.getClosedLoopFeedForward().getValueAsDouble());
    LogManager.addEntry("Shooter/AngleChanging/CloseLoopSP", () -> angleChangingMotor.getClosedLoopReference().getValueAsDouble());
  }
}
