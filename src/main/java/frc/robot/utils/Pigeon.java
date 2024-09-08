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
    public double getPigeonPitchDegree(){
        return pigeon.getPitch().getValue();
    }
    public Rotation2d getPigeonPitchRotation2d(){
        return Rotation2d.fromDegrees(getPigeonPitchDegree());
    }
    public double getPigeonRollDegree(){
        return pigeon.getRoll().getValue();
    }
    public Rotation2d getPigeonRollRotation2d(){
        return Rotation2d.fromDegrees(getPigeonRollDegree());
    }
    public double getXVelocityDPS(){
        return pigeon.getAngularVelocityXDevice().getValue();
    }
    public double getYVelocityDPS(){
        return pigeon.getAngularVelocityYDevice().getValue();
    }
    public double getZVelocityDPS(){
        return pigeon.getAngularVelocityZDevice().getValue();
    }
    public void resetPigeon(){
        pigeon.reset();
    }
}
