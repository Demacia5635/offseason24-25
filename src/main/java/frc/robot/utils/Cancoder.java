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
    public double getNonAbsPosition() {
        return cancoder.getPosition().getValue();
    }
    public Rotation2d getNonAbsRotation2d() {
        return Rotation2d.fromRotations(getNonAbsPosition());
    }
    public double getAbsRotation() {
        return cancoder.getAbsolutePosition().getValue();
    }
    public Rotation2d getAbsRotation2d() {
        return Rotation2d.fromRotations(getAbsRotation());
    }
    public double getVelocityRotation(){
        return cancoder.getVelocity().getValue();
    }
    public Rotation2d getVelocityRotation2d(){
        return Rotation2d.fromRotations(getVelocityRotation());
    }
    public void setOffset(double offset){
        canConfig.MagnetSensor.MagnetOffset = offset;
        cancoder.getConfigurator().apply(canConfig); 
    }
    public void setCanCoderClockwise(Boolean boolDirection){
        SensorDirectionValue direction = boolDirection ? SensorDirectionValue.Clockwise_Positive: SensorDirectionValue.CounterClockwise_Positive;
        canConfig.MagnetSensor.SensorDirection = direction;
        cancoder.getConfigurator().apply(canConfig);
    }
}
