package frc.robot.subsystems.chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Sysid.FeedForward_SVA;
import frc.robot.Sysid.Sysid;
import frc.robot.subsystems.chassis.ChassisConstants.SwerveModuleConstants;
import frc.robot.utils.Trapezoid;
import frc.robot.utils.Utils;
import static frc.robot.subsystems.chassis.ChassisConstants.*;

/**
 * Swerve Module
 */
public class SwerveModule implements Sendable {
    private final TalonFX moveMotor;
    private final TalonFX steerMotor;
    private final CANCoder absoluteEncoder;

    private final double angleOffset;
    private double pulsePerDegree;
    private double pulsePerMeter;

    private FeedForward_SVA MoveFFSlow;
    private FeedForward_SVA moveFFFast;
    private FeedForward_SVA moveFFFast2;

    private FeedForward_SVA steerFF;
    private Trapezoid steerTrapezoid;

    private static final int SteerPositionSlot = 1;
    private static final int SteerVelocitySlot = 0;

    double targetVelocity = 0;
    Rotation2d targetAngle = new Rotation2d();
    public String name;

    private double maxVelocityChange = DRIVE_ACCELERATION * CYCLE_DT;
    double maxSteerVelocityChange = STEER_ACCELERATION * CYCLE_DT;
    double MaxSteerClosedLoopError;

    private boolean useSteerPositionPID = true;
    public boolean debug = false;

    Chassis chassis;

    /**
     * Constructor
     * @param constants
     * @param chassis
     */
    public SwerveModule(SwerveModuleConstants constants, Chassis chassis) {
        this.chassis = chassis;
        // define motors and encoder
        moveMotor = new TalonFX(constants.moveMotorId);
        steerMotor = new TalonFX(constants.angleMotorId);
        moveMotor.configFactoryDefault();
        steerMotor.configFactoryDefault();
        absoluteEncoder = new CANCoder(constants.absoluteEncoderId);
        absoluteEncoder.configFactoryDefault();

        // set the controls
        // moveFF = new FeedForward_SVA_V2_VSQRT(constants.moveFF.KS, constants.moveFF.KV, constants.moveFF.KA,
        //     MOVE_KV2, MOVE_KVSQRT);
        MoveFFSlow = new FeedForward_SVA(constants.moveFFSlow.KS, constants.moveFFSlow.KV, constants.moveFFSlow.KA);
        moveFFFast = new FeedForward_SVA(constants.moveFFFast.KS, constants.moveFFFast.KV, constants.moveFFFast.KA);
        moveFFFast2 = new FeedForward_SVA(constants.moveFFFast2.KS, constants.moveFFFast2.KV, constants.moveFFFast2.KA);
        steerFF = new FeedForward_SVA(constants.steerFF.KS, constants.steerFF.KV, constants.steerFF.KA);
        setMovePID(0,constants.movePID.KP, constants.movePID.KI, constants.movePID.KD);
        setSteerPID(SteerVelocitySlot,constants.steerPID.KP, constants.steerPID.KI, constants.steerPID.KD);
        pulsePerDegree = constants.pulsePerDegree;
        pulsePerMeter = constants.pulsePerMeter;
        MaxSteerClosedLoopError = MAX_STEER_ERROR*pulsePerDegree;
        steerTrapezoid = new Trapezoid(MAX_STEER_VELOCITY, STEER_ACCELERATION);
        angleOffset = constants.steerOffset;
        // name
        name = (constants.moduleTranslationOffset.getX()>0?"Front":"Back") + 
               (constants.moduleTranslationOffset.getY()>0?"Left":"Right");
        debug = constants.moduleTranslationOffset.getX() < 0 && constants.moduleTranslationOffset.getY() >0;

        // set motors paramters
        moveMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setNeutralMode(NeutralMode.Brake);
        moveMotor.setInverted(false);
        steerMotor.setInverted(constants.inverted);
        moveMotor.setSelectedSensorPosition(0);
        steerMotor.setSelectedSensorPosition(getAngleDegrees()*pulsePerDegree);
        steerMotor.configClosedloopRamp(0.5);
        steerMotor.configOpenloopRamp(0.5);
        steerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);

        // set position PID at motor
        calculateSteerPositionPID(constants);
        setSteerPositionPID(constants);
        setSteerPID(SteerPositionSlot,constants.steerPositionPID.KP,constants.steerPositionPID.KI, constants.steerPositionPID.KD);


        // add the sysid for the module
//        SmartDashboard.putData(name + " Steer Sysid", (new Sysid(this::setSteerPower, this::getSteerVelocity, 0.1, 0.8, chassis)).getCommand());
    
    }

/**
 * calculate the steer position PID;
 * @param constants
 */
    private void calculateSteerPositionPID(SwerveModuleConstants constants) {
        double kp = steerFF.calculate(MAX_STEER_VELOCITY, MAX_STEER_VELOCITY)*1023.0/(90*pulsePerDegree);
        double ki = kp/10;
        double kd = ki/10;
        SmartDashboard.putNumber("pid position kp", kp);
        SmartDashboard.putNumber("pid position kd", kd);
        SmartDashboard.putNumber("pid position ki", ki);
    }

    /**
     * calculates and sets close loop params
     * @param constants
     */
    private void setSteerPositionPID(SwerveModuleConstants constants) {
        // KP - to provide MAX_VELOCITY (with no accelration) at 90 degrees error
        double kp = constants.steerPositionPID.KP;
        double ki = constants.steerPositionPID.KI;
        double kd = constants.steerPositionPID.KD;



        // set the maximum integral to provide 1.1*KS value
        steerMotor.configMaxIntegralAccumulator(SteerPositionSlot, 0.9*constants.steerFF.KS*1023/ki);
        steerMotor.configAllowableClosedloopError(SteerPositionSlot, MaxSteerClosedLoopError);
        // set integral zone to 0 when more than 10 degrees error
        steerMotor.config_IntegralZone(SteerPositionSlot, constants.INTEGRAL_ZONE*pulsePerDegree);
        steerMotor.configAllowableClosedloopError(SteerPositionSlot, MAX_STEER_ERROR*pulsePerDegree);
        if(useSteerPositionPID) {
            steerMotor.selectProfileSlot(SteerPositionSlot, 0);
        }
    }

    /**
     * Set the move motor PID
     * @param slot
     * @param kP
     * @param kI
     * @param kD
     */
    public void setMovePID(int slot, double kP, double kI, double kD) {
        moveMotor.config_kP(slot, kP);
        moveMotor.config_kI(slot, kI);
        moveMotor.config_kD(slot, kD);
    }

    /**
     * Set the Steer velocity PID
     * @param slot
     * @param kP
     * @param kI
     * @param kD
     */
    public void setSteerPID(int slot,double kP, double kI, double kD) {
        steerMotor.config_kP(slot, kP);
        steerMotor.config_kI(slot, kI);
        steerMotor.config_kD(slot, kD);
    }


    /**
     * Get the module angle from Talon encoder 
     * Use to make sure steer direction and gear ratio/pulse per degree are correct
     * @return angle based on talon encoder
     */
    public double steerTalonAngle() {
        return Utils.degrees(steerMotor.getSelectedSensorPosition()/pulsePerDegree);
    }
    /**
     * Get the talon motor degrees
     *  Use to make sure steer direction and gear ratio/pulse per degree are correct
     * @return the motor angle without the gear ratio
     */
    public double steerTalonRawAngle() {
        return steerMotor.getSelectedSensorPosition()*360/MOTOR_PULSES_PER_ROTATION;
    }

    /**
     * Returns the velocity of the module drive motor
     * @return Velocity in m/s
     */
    public double getVelocity() {
        return talonVelocityToVelocity(moveMotor.getSelectedSensorVelocity());
    }

    /**
     * Stops the module completely
     */
    public void stop() {
        setPower(0);
        setSteerPower(0);
    }

    /**
     * Sets the neutral mode of both motors
     */
    public void setNeutralMode(NeutralMode mode) {
        moveMotor.setNeutralMode(mode);
        steerMotor.setNeutralMode(mode);
    }

    /**
     * Sets the velocity of the module
     * @param v Velocity in m/s
     */

    double lastMoveA = 0;
    double lastMoveV = 0;

    public void setVelocity(double v) {
        targetVelocity = v;
        if(Math.abs(targetVelocity) < 0.03) {
            targetVelocity = 0;
        }
        double currentVelocity = getVelocity();
        /*
        double cv = currentVelocity;
        if(lastMoveA > 0 && currentVelocity < lastMoveV && currentVelocity < v) {
            currentVelocity = lastMoveV;
        } else if(lastMoveA < 0 && currentVelocity > lastMoveV && currentVelocity < v) {
            currentVelocity = lastMoveV;
        }
        */
        double tgtV = MathUtil.clamp(v, currentVelocity-maxVelocityChange, currentVelocity+maxVelocityChange);
        double ff = Math.abs(tgtV) > 2.5 ? moveFFFast.calculate(tgtV, currentVelocity):
                        MoveFFSlow.calculate(tgtV, currentVelocity);
        if (Math.abs(tgtV) > 3) ff = moveFFFast2.calculate(tgtV, currentVelocity);
    
        debug(" tgtV=" + tgtV + " cur V=" + currentVelocity + " ff=" + ff);
        moveMotor.set(ControlMode.Velocity, talonVelocity(tgtV), DemandType.ArbitraryFeedForward, ff);
        lastMoveA = tgtV - currentVelocity;
        lastMoveV = tgtV;
    }

     /**
     * Sets the power of the module
     * @param p Power in precent of the module (0%..100%)
     */
    public void setPower(double p) {
        moveMotor.set(ControlMode.PercentOutput, p);
    }

    /**
     * Get the module angle as Rotation2D
     * @return
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition() - angleOffset);
    }


    public double getAngleDegreesRaw() {
        return absoluteEncoder.getAbsolutePosition();
    }

    /**
     * Get the module angle in degrees in the -180 to 180 range
     * @return
     */
    public double getAngleDegrees() {
        return Utils.degrees(getAngle());
    }

    /**
     * Sets the rotational power of the module
     * @param p Power in precent of the module (0%..100%)
     */
    public void setSteerPower(double p) {
        steerMotor.set(ControlMode.PercentOutput, p);
    }

    /**
     * Returns the rotational velocity of the module
     * @return Velocity in deg/s
     */
    public double getSteerVelocity() {  
        return talonToSteerVelocity(steerMotor.getSelectedSensorVelocity());
    }

    /**
     * Sets the angular velocity of the module
     * @param v Velocity in deg/s
     */
    public void setSteerVelocity(double v, boolean withAcceleration) {
        steerMotor.selectProfileSlot(SteerVelocitySlot,0);
        double currentVelocity = getSteerVelocity();
        double tgtV = withAcceleration?MathUtil.clamp(v, currentVelocity-maxSteerVelocityChange, currentVelocity+maxSteerVelocityChange):v;
        double ff = steerFF.calculate(tgtV,currentVelocity);
        steerMotor.set(ControlMode.Velocity, talonSteerVelocity(tgtV), DemandType.ArbitraryFeedForward, ff);
    }

    /**
     * Set the module angle using position PID
     * @param angle
     */
    public void setAngleByPositionPID(Rotation2d angle) {
        steerMotor.selectProfileSlot(SteerPositionSlot,0);
//        if(targetAngle.equals(angle)) {
//            debug("set angle - same value - no change - " + " error=" + steerMotor.getClosedLoopError() + " power=" + steerMotor.getMotorOutputPercent() );
//        } else {
            targetAngle = angle;
            Rotation2d currentAngle = getAngle();
            double diff = Utils.degrees(angle.minus(currentAngle));
            double currentPos = steerMotor.getSelectedSensorPosition();
            double targetPos = currentPos + diff*pulsePerDegree;
            steerMotor.set(ControlMode.Position,targetPos);
//        }
    }

    public boolean atSteerAngle() {
        if(useSteerPositionPID) {
            return Math.abs(steerMotor.getClosedLoopError()) < MaxSteerClosedLoopError;
        } else {
            return Math.abs(Utils.degrees(getAngle().minus(targetAngle))) < MAX_STEER_ERROR;
        }

    }

    /**
     * Set module angle by Trapeziod/Velociyu
     * @param angle
     */
    public void setAngleByVelcoity(Rotation2d angle)  {
        targetAngle = angle;
        double diff = Utils.degrees(angle.minus(getAngle()));
        double v = 0;
        if(Math.abs(diff) > MAX_STEER_ERROR) {
            double cv = getSteerVelocity();
            v = steerTrapezoid.calculate(diff, cv, 0, debug);
        }
        setSteerVelocity(v, false);
    }

    /**
     * Debug message
     * @param s
     */
    private void debug(String s) {
        debug = false;
        if(debug){
          System.out.println(name + ": " + s);
        } 
    }


    /**
     * Returns the state of the module
     * @return Velocity in m/s
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Sets the state of the module
     */
    public void setState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        setVelocity(optimized.speedMetersPerSecond);
        if(useSteerPositionPID) {
            setAngleByPositionPID(optimized.angle);
        } else {
            setAngleByVelcoity(optimized.angle);
        }
    }

    /**
     * Returns the module position
     * @return Position relative to the field
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(moveMotor.getSelectedSensorPosition() / pulsePerMeter, getAngle());
    }

    public double talonVelocity(double v) {
        return v * pulsePerMeter / 10;
    }

    public double talonVelocityToVelocity(double v) {
            return v / pulsePerMeter * 10;
    }

    public double talonSteerVelocity(double v) {
            return v * pulsePerDegree / 10;
    }
    
    public double talonToSteerVelocity(double speed) {
            return speed / pulsePerDegree * 10;
    }

    public double getDistance() {
        return moveMotor.getSelectedSensorPosition()/pulsePerMeter;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("angle", () -> getAngleDegrees(), null);
        builder.addDoubleProperty("steer error", () -> steerMotor.getClosedLoopError(), null);
        builder.addDoubleProperty("velocity", this::getVelocity, null);
        builder.addDoubleProperty("Steer velocity", this::getSteerVelocity, null);
        builder.addDoubleProperty("desired angle", () -> targetAngle.getDegrees(), null);
        builder.addDoubleProperty("desired velocity", () -> targetVelocity, null);
        builder.addDoubleProperty("Steer Talon Angle", this::steerTalonAngle, null);
        builder.addDoubleProperty("Steer power", ()->steerMotor.getMotorOutputPercent(), null);
        builder.addDoubleProperty("distance", this::getDistance, null);
        builder.addDoubleProperty("Steer Error", ()->steerMotor.getClosedLoopError()/pulsePerDegree, null);
        builder.addDoubleProperty("Steer Acum", ()->steerMotor.getIntegralAccumulator()/pulsePerDegree, null);


    }

    public void configFactoryDefault() {
        steerMotor.configFactoryDefault();
        moveMotor.configFactoryDefault();
    }

    public void setSteerCoast() {
        steerMotor.setNeutralMode(NeutralMode.Coast);
    }
    

}