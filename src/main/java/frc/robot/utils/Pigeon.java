// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Pigeon {
    private Pigeon2 pigeon;
    public Pigeon(int ID, String CANBUS) {
        pigeon = new Pigeon2(ID, CANBUS);
    }
    public double getPigeonAngleDegree(){
        return pigeon.getAngle();
    }
    public Rotation2d getPigeonAngleRotation2d(){
        return Rotation2d.fromDegrees(getPigeonAngleDegree());
    }
    public double getPigeonAngleRotation(){
        return getPigeonAngleRotation2d().getRotations();
    }
    public double getPigeonPitchDegree(){
        return pigeon.getPitch().getValue();
    }
    public Rotation2d getPigeonPitchRotation2d(){
        return Rotation2d.fromDegrees(getPigeonPitchDegree());
    }
    public double getPigeonPitchRotation(){
        return getPigeonPitchRotation2d().getRotations();
    }
    public double getPigeonRollDegree(){
        return pigeon.getRoll().getValue();
    }
    public Rotation2d getPigeonRollRotation2d(){
        return Rotation2d.fromDegrees(getPigeonRollDegree());
    }
    public double getPigeonRollRotation(){
        return getPigeonRollRotation2d().getRotations();
    }
    public double getXVelocityDegreePerSecond(){
        return pigeon.getAngularVelocityXDevice().getValue();
    }
    public Rotation2d getXVelocityRotation2d(){
        return Rotation2d.fromDegrees(getXVelocityDegreePerSecond());
    }
    public double getXVelocityRotation(){
        return getXVelocityRotation2d().getRotations();
    }
    public double getYVelocityDegreePerSecond(){
        return pigeon.getAngularVelocityYDevice().getValue();
    }
    public Rotation2d getYVelocityRotation2d(){
        return Rotation2d.fromDegrees(getYVelocityDegreePerSecond());
    }
    public double getYVelocityRotation(){
        return getYVelocityRotation2d().getRotations();
    }
    public double getZVelocityDegreePerSecond(){
        return pigeon.getAngularVelocityZDevice().getValue();
    }
    public Rotation2d getZVelocityRotation2d(){
        return Rotation2d.fromDegrees(getZVelocityDegreePerSecond());
    }
    public double getZVelocityRotation(){
        return getZVelocityRotation2d().getRotations();
    }
    public void resetPigeon(){
        pigeon.reset();
    }
}
