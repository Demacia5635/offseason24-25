// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_CONFIGS;
import frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_VAR;
import frc.robot.Shooter.ShooterConstants.MOTOR_IDS;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.utils.ShooterUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import static frc.robot.Shooter.ShooterConstants.*;

public class AngleChanger extends SubsystemBase {
  
  private TalonFX angleChangingMotor;

  private TalonFXConfiguration config;
  private DutyCycleOut m_request;
  private VelocityVoltage velocityVoltage;
  private MotionMagicVoltage motionMagicVoltage;
  public STATE angleState;
  public double angle;


  /** Creates a new AngleChanging. */
  public AngleChanger() {

    angleChangingMotor = new TalonFX(MOTOR_IDS.ANGLE_CHANGING_ID, MOTOR_IDS.CANBUS);
    config = new TalonFXConfiguration();
    angleState = STATE.SPEAKER;

    config.Slot0.kP = ANGLE_CHANGING_CONFIGS.KP;
    config.Slot0.kI = ANGLE_CHANGING_CONFIGS.KI;
    config.Slot0.kD = ANGLE_CHANGING_CONFIGS.KD;
    config.Slot0.kS = ANGLE_CHANGING_CONFIGS.KS;
    config.Slot0.kV = ANGLE_CHANGING_CONFIGS.KV;
    config.Slot0.kA = ANGLE_CHANGING_CONFIGS.KA;

    m_request  = new DutyCycleOut(0.0).withUpdateFreqHz(FreqHz);
    velocityVoltage = new VelocityVoltage(0).withSlot(0).withUpdateFreqHz(FreqHz);
    motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0).withUpdateFreqHz(FreqHz); 

    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = -1; /*TODO change to real angle */

    config.MotionMagic.MotionMagicCruiseVelocity = ANGLE_CHANGING_VAR.ANGLE_CHANGING_MAX_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = ANGLE_CHANGING_VAR.ANGLE_CHANGING_MAX_Acceleration;
    config.MotionMagic.MotionMagicJerk = ANGLE_CHANGING_VAR.ANGLE_CHANGING_MAX_JERK;

    config.Feedback.SensorToMechanismRatio = ANGLE_CHANGING_GEAR_RATIO;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    angleChangingMotor.getConfigurator().apply(config);
    /*TODO add brake invert  */

    SmartDashboard.putData(this);
  }
   


  public void setMotorPower(double power){
    angleChangingMotor.setControl(m_request.withOutput(power));
  }

  public void goToAngle(double wantedAngle){
    double distance = ShooterUtils.angleToDistance(wantedAngle);
    angleChangingMotor.setControl(motionMagicVoltage.withPosition(distance));
    angle = wantedAngle;
  }

  public void angleChangingPID(double vel){
    angleChangingMotor.setControl(velocityVoltage.withVelocity(vel));
  }

  public double getAngleMotorVel(){
    return angleChangingMotor.getVelocity().getValue();
  }

  public void setAngle(double angle){
    this.angle = angle;
  }

  public double getAngle(){
    return angle;
  }

  public void setBaseAngle() {
    angleChangingMotor.setPosition(ANGLE_CHANGING_VAR.BASE_ANGLE);
  }

  /**
   * 
   * @return returns if reverse hard limit is closed
   * 
    */
  public boolean isMaxAngle(){
    return angleChangingMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      SmartDashboard.putData("reset angle", new InstantCommand(()-> setBaseAngle(), this).ignoringDisable(true));
      builder.addDoubleProperty("ShooterAngle", this::getAngle, null);
      builder.addDoubleProperty("angleMotorVer", this::getAngleMotorVel, null);
  }
}
