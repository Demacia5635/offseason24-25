package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
    public final TalonFX motor1;
    public final TalonFX motor2;

    public AnalogInput limitInput;
    Counter counter;

    // SimpleMotorFeedforward ff2 = new SimpleMotorFeedforward(Parameters.ks2,
    // Parameters.kv2, Parameters.ka2);
    public Intake() {


        motor1 = new TalonFX(IntakeDeviceID.MOTOR1);
        motor1.configFactoryDefault();
        motor1.setInverted(Parameters.IS_INVERTED);

        motor2 = new TalonFX(IntakeDeviceID.MOTOR2);
        motor1.configFactoryDefault();
        motor2.setInverted(Parameters.IS_INVERTED2);


        limitInput = new AnalogInput(IntakeDeviceID.LIGHT_LIMIT);
        limitInput.setAccumulatorInitialValue(0);

        counter = new Counter();
        // setting up the sensor id
        counter.setUpSource(IntakeConstants.IntakeDeviceID.COUNTER_ID);
        // setting up the coutner to count pulse duration from rising edge to falling
        // edge
        counter.setSemiPeriodMode(true);
        // make the counter do in samples
        counter.setSamplesToAverage(5);

        SmartDashboard.putData(this);

        setBrake();

        SmartDashboard.putData("Brake", new InstantCommand(
                () -> this.setBrake(), this).ignoringDisable(true));
        SmartDashboard.putData("Coast", new InstantCommand(
                () -> this.setCoast(), this).ignoringDisable(true));

     //   SmartDashboard.putData("Intake", new IntakeCommand(this));
     //   SmartDashboard.putData("Dispense", new DispenseCommand(this));
    }

    public void configDevices() {
        motor1.config_kP(0, Parameters.KP);
        motor1.config_kI(0, Parameters.KI);
        motor1.config_kD(0, Parameters.KD);
    }

    public double getpower() {
        double pMotor = motor1.getMotorOutputPercent();
        return pMotor;
    }

    public double getLimitVolt() {
        return limitInput.getVoltage();
    }

    public boolean isNotePresent() {
        if (getLimitVolt() < Parameters.NOT_PRESENCE_VOLTAGE) {
            return true;
        }
        return false;
    }

    public boolean isCriticalCurrent() {
        return motor1.getOutputCurrent() >= Parameters.CRITICAL_CURRENT;
    }

    public void setPower(double p1) {
//        DataLogManager.log(" Setting power to " + p1);
        motor1.set(ControlMode.PercentOutput, p1);
        motor2.set(ControlMode.PercentOutput, p1);
    }

    public void setPower(double p1, double p2) {
        //DataLogManager.log(" Setting power to " + p1 + "/" + p2);
        motor1.set(ControlMode.PercentOutput, p1);
        motor2.set(ControlMode.PercentOutput, p2);
    }

    public void setVelocity(double velocity) {
//        DataLogManager.log(" Setting Velocity to " + velocity);

        motor1.set(ControlMode.Velocity, velocity);
        motor2.set(ControlMode.Velocity, velocity);

    }

    public double getMotorCurrent() {
        return motor1.getOutputCurrent();
    }

    public TalonFX getIntakeMotor() {
        return motor1;
    }

    public double getEncoderPos() {
        return motor1.getSelectedSensorPosition();
    }

    public double getEncoderPos2() {
        return motor2.getSelectedSensorPosition();
    }

    public void setBrake() {
        System.out.println("brake");
        motor1.setNeutralMode(NeutralMode.Brake);
        motor2.setNeutralMode(NeutralMode.Brake);

    }

    public void setCoast() {
        System.out.println("coast");
        motor1.setNeutralMode(NeutralMode.Coast);
        motor2.setNeutralMode(NeutralMode.Coast);
    }

    public void stop() {
//        DataLogManager.log(" set Intake stop");
        motor1.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);

    }

    public static double deadband(double value) {
        if (Math.abs(value) < Parameters.DEADBAND) {
            return 0;
        }
        return value;
    }

    public static Translation2d getStickLeft(CommandXboxController controller) {
        return new Translation2d(deadband(controller.getLeftX()), deadband(controller.getLeftY()));
    }

    public static Translation2d getStickRight(CommandXboxController controller) {
        return new Translation2d(deadband(controller.getRightX()), deadband(controller.getRightY()));
    }

    public double getRadVelocity() {
        return (motor1.getSelectedSensorVelocity() * ConvertionParams.MOTOR_GEAR_RATIO
                / ConvertionParams.MOTOR_PULSES_PER_SPIN) * 2 * Math.PI;
    }

    public boolean isNote2() {
        double t = counter.getPeriod();
        t *= 1000;
        t -= 1;
        if (t > 0.8 || t <= 0) {
            return false;
        }
        t *= IntakeConstants.Parameters.COUNTER_M_PERA;
        t += IntakeConstants.Parameters.COUNTER_B_PERA;
        return t < 300;
    }

    public Command getActivateIntakeCommand() {
        return new StartEndCommand(()->setPower(1), ()->setPower(0), this);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("is note", this::isNote2, null);
        builder.addDoubleProperty("Limit switch voltage intake", this::getLimitVolt, null);
        builder.addDoubleProperty("Intake Power", ()->motor1.getMotorOutputPercent(), null);
    }

}