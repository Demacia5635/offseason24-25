package frc.robot.chassis.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.chassis.ChassisConstants.SwerveModuleConstants;
import frc.robot.utils.LogManager;
import frc.robot.utils.TalonMotor;

import static frc.robot.chassis.ChassisConstants.SICLE_CAUNT;
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



        moveMotor = new TalonMotor(constants.driveConfig);
        steerMotor = new TalonMotor(constants.steerConfig);
        absoluteEncoder = new CANcoder(constants.absoluteEncoderId);

        // set the controls
        // moveFF = new FeedForward_SVA_V2_VSQRT(constants.moveFF.KS, constants.moveFF.KV, constants.moveFF.KA,
        //     MOVE_KV2, MOVE_KVSQRT);
        angleOffset = constants.steerOffset;
        
        canCOnfig = new CANcoderConfiguration();

        canCOnfig.MagnetSensor.MagnetOffset = angleOffset;

        absoluteEncoder.getConfigurator().apply(canCOnfig);
        // name
        debug = constants.moduleTranslationOffset.getX() < 0 && constants.moduleTranslationOffset.getY() >0;

        // set motors paramters
        moveMotor.setBrake(true);
        steerMotor.setBrake(true);
        moveMotor.setPosition(0);
        steerMotor.setPosition(getAngleDegreesRaw());

        SmartDashboard.putData(name, this);

        LogManager.addEntry(name + "/angle", this::getSteerTalonAngle);


    }

    @Override
    public void periodic() {
        if(caunt >= SICLE_CAUNT){
            //steerMotor.setPosition(getAbsDegrees().getDegrees());
            caunt =0;
        }
        caunt ++;
        
        
    }





    /**
     * Get the module angle from Talon encoder 
     * Use to make sure steer direction and gear ratio/pulse per degree are correct
     * @return angle based on talon encoder
     */
    public double getSteerTalonAngle() {
        return steerMotor.getCurrentPositionAsDegrees();
    }


    /**
     * Returns the velocity of the module drive motor
     * @return Velocity in m/s
     */
    public double getVelocity() {
        return moveMotor.getCurrentVelocityInMS(WHEEL_DIAMETER/2);
    }
    /**
   * set position to drive to in rotations in radians
   */
    public void setSteerPosition(Rotation2d angle){
        steerMotor.setMotorPosition(angle);
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


    public double getAngleDegreesRaw() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Get the module angle in degrees in the -180 to 180 range
     * @return
     */
    public double getAngleDegrees() {
        return getAbsDegrees().getDegrees();
    }

    /**
     * Sets the rotational power of the module
     * @param p Power in precent of the module (0%..100%)
     */
    public void setSteerPower(double p) {
        steerMotor.setDuty(p);
        System.out.println(name + ": " + steerMotor.getCurrentVelocityDegrees());
    }
    /**
     * Sets velosity to the module  in meters per secons
     */
    public void setSteerVelocity(double v) {
        steerMotor.setVelocityMPS(v,WHEEL_DIAMETER/2);
    }

    /**
     * Returns the rotational velocity of the module
     * @return Velocity in degrees per second
     */
    public double getSteerVelocity() {  
        return steerMotor.getCurrentVelocity().getDegrees();
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
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAbsDegrees());
        setSteerPosition(optimized.angle);
        setVelocity(optimized.speedMetersPerSecond);
    }

    /**
     * Returns the module position
     * @return Position relative to the field
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(moveMotor.getCurrentPosition().getDegrees(), getAbsDegrees());
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