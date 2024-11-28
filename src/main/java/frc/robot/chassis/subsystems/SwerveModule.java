package frc.robot.chassis.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.chassis.ChassisConstants.SwerveModuleConstants;
import frc.robot.utils.TalonMotor;

import static frc.robot.chassis.ChassisConstants.MOTOR_ROTATION_PER_METER;
import static frc.robot.chassis.ChassisConstants.WHEEL_DIAMETER;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;


/**
 * Swerve Module
 */
public class SwerveModule extends SubsystemBase {
    private final TalonMotor moveMotor;
    private final TalonMotor steerMotor;


    private final CANcoder absoluteEncoder;

    private final double angleOffset;

    private CANcoderConfiguration canCOnfig;

    double targetVelocity = 0;
    Rotation2d targetAngle = new Rotation2d();
    public String name;

    public boolean debug = false;

    Chassis chassis;

    double caunt = 0;

    /**
     * Constructor
     * @param constants
     * @param chassis
     */
    public SwerveModule(SwerveModuleConstants constants, Chassis chassis) {
        this.chassis = chassis;
        // define motors and encoder
        name = (constants.moduleTranslationOffset.getX()>0?"Front":"Back") + 
               (constants.moduleTranslationOffset.getY()>0?"Left":"Right");
        //debug = constants.moduleTranslationOffset.getX()>0&& constants.moduleTranslationOffset.getY()<0; 

        moveMotor = new TalonMotor(constants.driveConfig);
        steerMotor = new TalonMotor(constants.steerConfig);
        absoluteEncoder = new CANcoder(constants.absoluteEncoderId, Constants.CANBUS);

        // set the controls
        // moveFF = new FeedForward_SVA_V2_VSQRT(constants.moveFF.KS, constants.moveFF.KV, constants.moveFF.KA,
        //     MOVE_KV2, MOVE_KVSQRT);
        angleOffset = constants.steerOffset;
        
        canCOnfig = new CANcoderConfiguration();

//        canCOnfig.MagnetSensor.MagnetOffset = angleOffset;

        absoluteEncoder.getConfigurator().apply(canCOnfig);

        

        // set motors paramters
        moveMotor.setBrake(true);
        steerMotor.setBrake(true);
        moveMotor.setPosition(0);
        steerMotor.setPosition(getAngleRaw() - angleOffset);

        SmartDashboard.putData(name, this);

    }

    @Override
    public void periodic() {
        /*
        if(caunt >= SICLE_CAUNT){
            //steerMotor.setPosition(getAbsDegrees().getDegrees());
            caunt =0;
        }
        caunt ++;
        
        */
    }



    /**
     * Returns the velocity of the module drive motor
     * @return Velocity in m/s
     */
    public double getVelocity() {
        return moveMotor.getCurrentVelocityInMS(WHEEL_DIAMETER/2);
    }
    /**
   * set position to drive to in rotations
   */
    public void setSteerPosition(double tgt){
//        double cur = steerMotor.getPosition().getValue();
//        double off = MathUtil.inputModulus(angle.getRotations()-cur, -0.5, 0.5);
        steerMotor.setMotorPosition(tgt);
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
        moveMotor.setVelocityMPS(targetVelocity, WHEEL_DIAMETER/2);

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


    public Rotation2d getAbsDegrees() {
        return Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValue());
    }

    public double getAngleRaw() {

        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Get the module angle in degrees in the -180 to 180 range
     * @return
    public double getAngleDegrees() {
        return getAbsDegrees().getDegrees();
    }

    /**
     * Sets the rotational power of the module
     * @param p Power in precent of the module (0%..100%)
     */
    public void setSteerPower(double p) {
        steerMotor.setDuty(p);
    }
    public void setSteerVoltage(double v) {
        steerMotor.setVoltage(v);
    }
    /**
     * Sets velosity to the module  in rps
     */
    public void setSteerVelocity(double v) {
        steerMotor.setVelocity(v);
    }

    /**
     * Returns the rotational velocity of the module
     * @return Velocity in degrees per second
     */
    public double getSteerVelocity() {  
        return steerMotor.getCurrentVelocity();
    }

    /**
     * Returns the state of the module
     * @return Velocity in m/s
     * @return pose in degrees in m/s
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAbsDegrees());
    }

    /**
     * Sets the state of the module
     */
    public void setState(SwerveModuleState state) {
        double cur = steerMotor.getPosition().getValue();
        double tgt = state.angle.getRotations();
        double dif = tgt - cur;
        double v = state.speedMetersPerSecond;
        dif = MathUtil.inputModulus(dif, -0.5, 0.5);
        if(dif > 0.25) {
            v = -v;
            dif = dif-0.5;
        } else if(dif < -0.25) {
            v = -v;
            dif = dif + 0.5;
        }

        if(v == 0) {
            steerMotor.set(0);
        } else {
            // if(debug) System.out.println("id = " + steerMotor.getDeviceID() + 
            //     " v=" + state.speedMetersPerSecond + " / " + v +
            //     " state = " + state.angle.getRotations() + 
            //     " cur=" + cur +
            //     " tgt=" + (cur + dif) + 
            //     " dif=" + dif);
            setSteerPosition(cur + dif);
        }
        System.out.println(name + " " + v);
        setVelocity(v);
    }

    public Rotation2d getModuleAngle(){
        return Rotation2d.fromRotations(steerMotor.getPosition().getValue());
    }

    /**
     * Returns the module position
     * @return Position relative to the field
     */
    double lastPos = 0;
    public SwerveModulePosition getModulePosition() {
        double pos = moveMotor.getPosition().getValue()*MOTOR_ROTATION_PER_METER;
        Rotation2d rot = getModuleAngle();
        if(debug && lastPos != pos) {
            System.out.println(name + " pos=" + pos + " angle="  + rot.getDegrees());
            lastPos = pos;
        } 
        return new SwerveModulePosition(pos,rot);
    }

   

    @Override
    public void initSendable(SendableBuilder builder) {
//        builder.addDoubleProperty("desired angle", () -> targetAngle.getDegrees(), null);
//        builder.addDoubleProperty("desired velocity", () -> targetVelocity, null);
//        builder.addDoubleProperty("Steer Talon Angle", this::steerTalonAngle, null);
//        builder.addDoubleProperty("distance", this::getDistance, null);
        builder.addDoubleProperty("abs encoder", () -> absoluteEncoder.getAbsolutePosition().getValue(), null);
        builder.addDoubleProperty("abs encoder in angle", ()-> Rotation2d.fromRotations(absoluteEncoder.getAbsolutePosition().getValue()).getDegrees(), null);
    }


    public void setSteerCoast() {
        steerMotor.setBrake(false);
    }
    

}