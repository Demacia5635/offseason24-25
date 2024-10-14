package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class Cancoder {
    private CANcoder cancoder;
    private CANcoderConfiguration canConfig;
    public Cancoder(int ID, String CANBUS) {
        cancoder = new CANcoder(ID, CANBUS);
        canConfig = new CANcoderConfiguration();
        cancoder.getConfigurator().apply(canConfig);
    }
    /**when the cancoder opens its start at the absolute position
     * @return the none absolute amaunt of rotations the motor did in rotation2d
    */
    public Rotation2d getNonAbsRotation2d() {
        return Rotation2d.fromRotations(cancoder.getPosition().getValueAsDouble());
    }
    /**
     * @return the absolute amaunt of rotations the motor did in rotation2d
     */
    public Rotation2d getAbsRotation2d() {
        return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble());
    }
    /** 
     * @return the amount of rotations the motor do per second in Rotation2d
     */
    public Rotation2d getVelocityRotation2dPerSec(){
        return Rotation2d.fromRotations(cancoder.getVelocity().getValueAsDouble());
    }
    /** set the offset of the cancoder to offset
     * subtracts from the position the offset at all time
     * @param 
     */
    public void setOffset(Rotation2d offset){
        canConfig.MagnetSensor.MagnetOffset = offset.getRotations();
        cancoder.getConfigurator().apply(canConfig); 
    }
    /** set the diraction of the cancoder
     * when changing the diraction the position will moltiply by minus one at all time
     * @param
     */
    public void setCanCoderClockwise(Boolean boolDirection){
        SensorDirectionValue direction = boolDirection ? SensorDirectionValue.Clockwise_Positive: SensorDirectionValue.CounterClockwise_Positive;
        canConfig.MagnetSensor.SensorDirection = direction;
        cancoder.getConfigurator().apply(canConfig);
    }
}
