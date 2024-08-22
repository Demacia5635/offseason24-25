package frc.robot.subsystems.amp;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.amp.AmpConstants.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utils.TrapezoidCalc;
import frc.robot.Robot;
import frc.robot.Sysid.Sysid;
import frc.robot.Sysid.Sysid.Gains;
import frc.robot.commands.amp.AmpIntake2;
import frc.robot.commands.amp.AmpIntakeShoot;

public class Amp extends SubsystemBase {
    public final TalonFX armMotor;
    public final TalonSRX lockMotor;
    public final CANSparkMax intakeMotor;
    public SparkAnalogSensor noteSensor;
    public int noteCount;
    public DigitalInput positionSensor;
    public boolean isLocked = false;
    public boolean enable = false;
    public double armEncoderOffset;

    // periodic data
    double targteAngle = Parameters.ARM_SENSOR_POSITION_ANGLE;
    boolean isLocking = true;
    boolean isUnlocking = false;
    double lockStartTime;
    TrapezoidCalc trap = new TrapezoidCalc();

    public Amp() {
        armMotor = new TalonFX(AmpDeviceID.ARM_MOTOR_ID);
        armMotor.configFactoryDefault();
        armMotor.setInverted(true);
        armMotor.setNeutralMode(NeutralMode.Coast);
        armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0.2));
        setArmAngle(Parameters.ARM_HOME_POSITION_ANGLE);
        configArmPID();

        positionSensor = new DigitalInput(AmpConstants.AmpDeviceID.MAGNETIC_SENSOR_ID);
        lockMotor = new TalonSRX(AmpConstants.AmpDeviceID.LOCK_MOTOR_ID);
        lockMotor.configFactoryDefault();
        lockMotor.setInverted(true);

        intakeMotor = new CANSparkMax(AmpDeviceID.INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.getEncoder().setPosition(0);
        intakeMotor.setInverted(true);

        setIntakeBrake();

        noteSensor = intakeMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute);

        noteCount = 0;
        SmartDashboard.putData(this);

    }

    public void configArmPID() {
        armMotor.config_kP(0, Parameters.ARM_KP);
        armMotor.config_kI(0, Parameters.ARM_KI);
        armMotor.config_kD(0, Parameters.ARM_KD);
    }

    public double getIntakeRev() {
        return intakeMotor.getEncoder().getPosition() / ConvertionParams.NEO_PULES_PER_REV
                * ConvertionParams.INTAKE_GEAR_RATIO;
    }

    public void setLockPower(double power) {
        lockMotor.set(ControlMode.PercentOutput, power);
    }

    public double getLockCurrent() {
        return lockMotor.getSupplyCurrent();
    }

    public boolean getPositionSensor() {
        return positionSensor.get();
    }

    public void setArmPower(double p) {
        armMotor.set(ControlMode.PercentOutput, p);
    }

    public double getArmPower() {
        return armMotor.getMotorOutputPercent();
    }

    /**
     * 
     * @return true if the arm is fully closed
     */
    public boolean isAtPositionSensor() {
        return (!getPositionSensor());
    }

    /**
     * 
     * @return true if the arm is fully open
     */
    public boolean isAtUpperPosition() {
        return Math.abs(getArmAngle() - Parameters.ARM_UPPER_POSITION_ANGLE) < Parameters.ARM_POSITION_ERROR;
    }

    public double getNoteSensorVolt() {
        return noteSensor.getVoltage();
    }

    public boolean isSensingNote() {
        return getNoteSensorVolt() < Parameters.NOTE_VOLTAGE;
    }

    public void resetNoteCounts() {
        noteCount = 0;
    }

    /**
     * 
     * @param last the last output of the didNotePass function
     * @return true if the note is inside and false if not
     */
    public boolean isNoteThere(boolean last) {
        if ((isSensingNote() == true) && (last == false)) {
            noteCount++;
        }
        return noteCount % 2 != 0;
    }

    public void setArmBrake() {
        armMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setArmCoast() {
        armMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void ArmStop() {
        armMotor.set(ControlMode.PercentOutput, 0);
    }

    public static double deadband(double value) {
        return Math.abs(value) < Parameters.Deadband ? 0 : value;
    }

    public double getArmVel() {
        return (armMotor.getSelectedSensorVelocity() * 10) / ConvertionParams.PULSE_PER_RAD;
    }

    public double getArmAngle() {
        return (armMotor.getSelectedSensorPosition() - armEncoderOffset) / ConvertionParams.PULSE_PER_RAD;
    }

    public void setArmAngle(double radians) {
        armEncoderOffset = armMotor.getSelectedSensorPosition() - radians * ConvertionParams.PULSE_PER_RAD;
        // armMotor.setSelectedSensorPosition(radians*ConvertionParams.PULSE_PER_RAD);
    }

    public double getArmCurrent() {
        return armMotor.getSupplyCurrent();
    }

    public double ArmFF(double wantedAnglerVel) {
        double rad = getArmAngle();

        return (armStatesParameters.KS +
                wantedAnglerVel * armStatesParameters.KV +
                (wantedAnglerVel - getArmVel()) * armStatesParameters.KA +
                armStatesParameters.Kcos * Math.cos(rad));

    }

    public void setVel(double wantedAnglerVel) {
        armMotor.set(ControlMode.Velocity, wantedAnglerVel * ConvertionParams.MOTOR_PULSES_PER_ANGLE / 10,
                DemandType.ArbitraryFeedForward, ArmFF(wantedAnglerVel));
    }

    @Override
    public void periodic() {
        super.periodic();
        if (!enable && Robot.robot.isEnabled()) {
            unlock();
            isLocked = true;
            setArmBrake();
            setArmAngle(Parameters.ARM_HOME_POSITION_ANGLE);
            goToSensor();
            enable = true;
        } else if (!Robot.robot.isEnabled()) {
            enable = false;
            setArmCoast();
        }
        if (isAtPositionSensor()) {
            setArmAngle(Parameters.ARM_SENSOR_POSITION_ANGLE);
        }
        //lockPeriodic();
        //armPeriodic();

    }

    public boolean isIntakePushingNote() {
        return intakeMotor.getOutputCurrent() >= Parameters.CRITICAL_CURRENT;
    }

    public void setIntakePower(double p) {
        intakeMotor.set(p);
    }

    public double getIntakeMotorCurrent() {
        return intakeMotor.getOutputCurrent();
    }

    public void setIntakeBrake() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setIntakeCoast() {
        intakeMotor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        Command cmd = new Sysid(new Gains[] { Gains.KS, Gains.KV, Gains.KA, Gains.KCos }, this::setArmPower,
                this::getArmVel, this::getArmAngle, null, 0.12, 0.3, 3, 0.5, 0.5, this).getCommand();
        SmartDashboard.putData("Amp SYSID", cmd);
        builder.addDoubleProperty("Arm Angle", this::getArmAngle, null);
        builder.addDoubleProperty("Lock Current", this::getLockCurrent, null);
        builder.addBooleanProperty("Sensing Note", this::isSensingNote, null);
        builder.addDoubleProperty("Note Sensor Volt", this::getNoteSensorVolt, null);
        builder.addBooleanProperty("Position Sensor", this::isAtPositionSensor, null);
        builder.addBooleanProperty("Upper Position", this::isAtUpperPosition, null);
        builder.addDoubleProperty("Arm Current", this::getArmCurrent, null);

        SmartDashboard.putData("Brake Arm", new InstantCommand(
                () -> this.setArmBrake(), this).ignoringDisable(true));
        SmartDashboard.putData("Coast Arm", new InstantCommand(
                () -> this.setArmCoast(), this).ignoringDisable(true));
        SmartDashboard.putData("release brake", new InstantCommand(
                () -> setLockPower(Parameters.UNLOCK_POWER), this));

    }

    // lock as periodic
    public void lock() {
        if (!isLocked || isUnlocking) {
            isUnlocking = false;
            isLocking = true;
            isLocked = false;
            lockStartTime = Timer.getFPGATimestamp();
        }
    }

    public void unlock() {
        if (isLocked || isLocking) {
            isLocking = false;
            isLocked = true;
            isUnlocking = true;
            lockStartTime = Timer.getFPGATimestamp();
        }
    }

    public boolean isLocked() {
        return isLocked;
    }

    private void lockPeriodic() {
        if (Robot.robot.isEnabled()) {
            if (isLocking) {
                if (getLockCurrent() > Parameters.LOCK_MAX_AMPER) {
                    isLocked = true;
                    isLocking = false;
                    isUnlocking = false;
                    setLockPower(0);
                } else {
                    setLockPower(Parameters.LOCK_POWER);
                }
            } else if (isUnlocking) {
                if (Timer.getFPGATimestamp() > lockStartTime + Parameters.UNLOCK_TIME) {
                    isUnlocking = false;
                    isLocked = false;
                    isLocking = false;
                    setLockPower(0);
                } else {
                    setLockPower(Parameters.UNLOCK_POWER);
                }

            }
        }
    }

    private boolean isArmInPosition() {
        return Math.abs(getArmAngle() - targteAngle) < Parameters.ARM_POSITION_ERROR;
    }

    public void goToSensor() {
        targteAngle = Parameters.ARM_SENSOR_POSITION_ANGLE;
    }

    public void goToUpperPosition() {
        targteAngle = Parameters.ARM_UPPER_POSITION_ANGLE;
    }

    public void goToClimbReadyPosition() {
        targteAngle = Parameters.ARM_CLIMB_READY_POSITION_ANGLE;
    }

    public void climb() {
        targteAngle = Parameters.ARM_CLIMB_POSITION_ANGLE;
    }

    private void armPeriodic() {
        if (Robot.robot.isEnabled()) {
            if (getArmCurrent() > Parameters.ARM_DOWN_CURRENT_LIMIT && getArmPower() < 0) {
                setArmAngle(Parameters.ARM_HOME_POSITION_ANGLE);
                setArmPower(0);
            } else if (isArmInPosition()) {
                if (!isLocked) {
                    if (targteAngle == Parameters.ARM_CLIMB_POSITION_ANGLE) {
                        setArmPower(Parameters.ARM_CLIMB_STOP_POWER);
                        lock();
                    } else {
                        setVel(0);
                        lock();
                    }
                } else {
                    setArmPower(0);
                }
            } else {
                double currentAngle = getArmAngle();
                if (isLocked) {
                    setVel(0);
                    unlock();
                } else if (targteAngle > currentAngle) {
                    if (currentAngle > Parameters.ARM_CLIMB_READY_POSITION_ANGLE) {
                        if (getArmCurrent() > Parameters.ARM_CLIMB_MAX_POWER_TRIGGER_CURRENT) {
                            setArmPower(Parameters.ARM_CLIMB_MAX_POWER);
                        } else {
                            setArmPower(Parameters.ARM_CLIMB_INITIAL_POWER);
                        }
                    } else {
                        double v = trap.trapezoid(getArmVel(),
                                Parameters.MAX_ARM_VEL_OPEN, 0,
                                Parameters.MAX_ARM_ACCEL_OPEN, targteAngle - currentAngle);
                        setVel(v);
                    }
                } else {
                    setArmPower(Parameters.ARM_DOWN_POWER);
                }
            }
        }
    }

    // commands
    public Command getReadyCommand(Intake intake) {
        return new InstantCommand(() -> goToSensor(), this).andThen(
                new WaitUntilCommand(() -> isArmInPosition()),
                new InstantCommand(() -> goToUpperPosition(), this),
                new WaitUntilCommand(() -> isArmInPosition()));
    }

    public Command getCancelCommand() {
        return new InstantCommand(() -> goToSensor());

    }

    public Command getShootCommand() {
        return new AmpIntakeShoot(this);
    }

    public Command getClimbReadyCommand() {
        return new InstantCommand(() -> goToClimbReadyPosition());
    }

    public Command getClimbCommand() {
        return new InstantCommand(() -> climb());
    }

    public void setSnowBlowerPower( double power) {
       lockMotor.set(TalonSRXControlMode.PercentOutput, power);
    }

    public double getSnowBlowerAmper() {
        return lockMotor.getSupplyCurrent();
    }

    public boolean getNeedStop() {
        return lockMotor.getSupplyCurrent() > AmpConstants.Parameters.MAX_SNOWBLOWER_VOLT;
    }


}