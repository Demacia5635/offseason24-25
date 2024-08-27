package frc.robot.chassis.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Sysid.FeedForward_SVA;
import frc.robot.chassis.ChassisConstants.SwerveModuleConstants;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;
import frc.robot.utils.Trapezoid;
import frc.robot.utils.Utils;
import static frc.robot.chassis.ChassisConstants.*;


import com.ctre.phoenix6.hardware.CANcoder;


/**
 * Swerve Module
 */
public class SwerveModule implements Sendable {
    private final TalonMotor moveMotor;
    private final TalonMotor steerMotor;

    private final TalonConfig moveMotorConfig;
    private final TalonConfig steerMotorConfig;

    private final CANcoder absoluteEncoder;

    private final double angleOffset;
    private double pulsePerDegree;
    private double pulsePerMeter;

    private FeedForward_SVA MoveFFSlow;
    private FeedForward_SVA moveFFFast;
    private FeedForward_SVA moveFFFast2;

    private FeedForward_SVA steerFF;
    private Trapezoid steerTrapezoid;



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
        moveMotorConfig = new TalonConfig(constants.moveMotorId,"rio" , "moveMotor");
        steerMotorConfig = new TalonConfig(constants.angleMotorId,"rio" , "steerMotor");

        moveMotorConfig.withPID(constants.movePID.KP, constants.movePID.KI, constants.movePID.KD,constants.moveFFFast2.KS, constants.moveFFFast2.KV, constants.moveFFFast2.KA,0);
        steerMotorConfig.withPID(constants.steerPID.KP, constants.steerPID.KI, constants.steerPID.KD,constants.steerFF.KS, constants.steerFF.KV, constants.steerFF.KA,0);

        moveMotor = new TalonMotor(moveMotorConfig);
        steerMotor = new TalonMotor(steerMotorConfig);
        absoluteEncoder = new CANcoder(constants.absoluteEncoderId);

        // set the controls
        // moveFF = new FeedForward_SVA_V2_VSQRT(constants.moveFF.KS, constants.moveFF.KV, constants.moveFF.KA,
        //     MOVE_KV2, MOVE_KVSQRT);
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
        moveMotor.setBrake(true);
        steerMotor.setBrake(true);
        moveMotor.setInverted(false);
        steerMotor.setInverted(constants.inverted);
        moveMotor.setEncoderPosition(0);
        steerMotor.setEncoderPosition(getAngleDegrees()*pulsePerDegree);

        // set position PID at motor
        calculateSteerPositionPID(constants);


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




    /**
     * Get the module angle from Talon encoder 
     * Use to make sure steer direction and gear ratio/pulse per degree are correct
     * @return angle based on talon encoder
     */
    public double steerTalonAngle() {
        return steerMotor.getCurrentPosition().getDegrees();
    }


    /**
     * Returns the velocity of the module drive motor
     * @return Velocity in m/s
     */
    public double getVelocity() {
        return talonVelocityToVelocity(moveMotor.getCurrentVelocity());
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
    public void setBrake(Boolean brake) {
        moveMotor.setBrake(brake);
        steerMotor.setBrake(brake);
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
        moveMotor.setVelocity(talonVelocity(tgtV));
        lastMoveA = tgtV - currentVelocity;
        lastMoveV = tgtV;
    }

     /**
     * Sets the power of the module
     * @param p Power in precent of the module (0%..100%)
     */
    public void setPower(double p) {
        moveMotor.setDuty(p);
    }

    /**
     * Get the module angle as Rotation2D
     * @return
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition().getValueAsDouble() - angleOffset);
    }


    public double getAngleDegreesRaw() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Get the module angle in degrees in the -180 to 180 range
     * @return
     */
    public double getAngleDegrees() {
        return getAngle().getDegrees();
    }

    /**
     * Sets the rotational power of the module
     * @param p Power in precent of the module (0%..100%)
     */
    public void setSteerPower(double p) {
        steerMotor.setDuty(p);
    }

    /**
     * Returns the rotational velocity of the module
     * @return Velocity in deg/s
     */
    public double getSteerVelocity() {  
        return talonToSteerVelocity(steerMotor.getVelocity().getValueAsDouble());
    }

    /**
     * Sets the angular velocity of the module
     * @param v Velocity in deg/s
     */
    public void setSteerVelocity(double v, boolean withAcceleration) {
        double currentVelocity = getSteerVelocity();
        double tgtV = withAcceleration?MathUtil.clamp(v, currentVelocity-maxSteerVelocityChange, currentVelocity+maxSteerVelocityChange):v;
        steerMotor.setVelocity(talonSteerVelocity(tgtV));
    }

    /**
     * Set the module angle using position PID
     * @param angle
     */
    public void setAngleByPositionPID(Rotation2d angle) {
            targetAngle = angle;
            Rotation2d currentAngle = getAngle();
            double diff = angle.minus(currentAngle).getDegrees();
            double currentPos = steerMotor.getCurrentPosition().getDegrees();
            double targetPos = currentPos + diff*pulsePerDegree;
            steerMotor.setMotorPosition(targetPos);
//        }
    }

    public boolean atSteerAngle() {
        if(useSteerPositionPID) {
            return Math.abs(steerMotor.getClosedLoopError().getValueAsDouble()) < MaxSteerClosedLoopError;
        } else {
            return Math.abs(getAngle().minus(targetAngle).getDegrees()) < MAX_STEER_ERROR;
        }

    }

    /**
     * Set module angle by Trapeziod/Velociyu
     * @param angle
     */
    public void setAngleByVelcoity(Rotation2d angle)  {
        targetAngle = angle;
        double diff = angle.minus(getAngle()).getDegrees();
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
        return new SwerveModulePosition(moveMotor.getCurrentPosition().getDegrees() / pulsePerMeter, getAngle());
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
        return moveMotor.getCurrentPosition().getDegrees()/pulsePerMeter;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("angle", () -> getAngleDegrees(), null);
        builder.addDoubleProperty("velocity", this::getVelocity, null);
        builder.addDoubleProperty("Steer velocity", this::getSteerVelocity, null);
        builder.addDoubleProperty("desired angle", () -> targetAngle.getDegrees(), null);
        builder.addDoubleProperty("desired velocity", () -> targetVelocity, null);
        builder.addDoubleProperty("Steer Talon Angle", this::steerTalonAngle, null);
        builder.addDoubleProperty("distance", this::getDistance, null);


    }


    public void setSteerCoast() {
        steerMotor.setBrake(false);
    }
    

}