// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class DemaciaKinematics extends SwerveDriveKinematics {

    Translation2d[] moduleTranslation;
    double maxVelocity = 3.8;
    double radius = 0.3;


    public DemaciaKinematics(Translation2d... moduleTranslationsMeters){
        super(moduleTranslationsMeters);
        this.moduleTranslation = moduleTranslationsMeters;
    }
    public Translation2d[] getModulesTranslation()
    {
        return this.moduleTranslation;
    }
    
   @Override
  public Twist2d toTwist2d(SwerveDriveWheelPositions start, SwerveDriveWheelPositions end) {
    
    SwerveModulePosition[] newPositions = new SwerveModulePosition[start.positions.length];
    for (int i = 0; i < start.positions.length; i++) {
      SwerveModulePosition startModule = start.positions[i];
      SwerveModulePosition endModule = end.positions[i];
      newPositions[i] =
          new SwerveModulePosition(
            endModule.distanceMeters - startModule.distanceMeters,
            (endModule.angle.plus(startModule.angle)).div(2));
    }
    return super.toTwist2d(newPositions);
  }

  public SwerveModuleState[] toSwerveModuleStatesWithAccel(ChassisSpeeds chassisSpeeds, Translation2d currentVelocity){
    Translation2d nextVel = getNextWantedVel(currentVelocity,
     new Translation2d(chassisSpeeds.vxMetersPerSecond,chassisSpeeds.vyMetersPerSecond));
    return toSwerveModuleStates(new ChassisSpeeds(nextVel.getX(),nextVel.getY(),
    chassisSpeeds.omegaRadiansPerSecond),currentVelocity);
  }
  
  private Translation2d getNextWantedVel(Translation2d currentVel,Translation2d finalWantedVel){
    Translation2d accel = currentVel.times(-1).plus(finalWantedVel);
    if(accel.getNorm() <= radius) return finalWantedVel;
    Rotation2d alpha = accel.getAngle();
    double nextWantedNorm = Math.sqrt(square(radius) + square(currentVel.getNorm())
     - (currentVel.getNorm() * radius * 2)
      * alpha.getCos()); 
    Rotation2d nextWantedAngle = Rotation2d.fromRadians(Math.asin((nextWantedNorm*alpha.getSin())/radius));
    return new Translation2d(nextWantedNorm,nextWantedAngle);
    
  }

  private double square(double n){
    return Math.pow(n, 2);
  }


  public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, Translation2d currentVelocity, Rotation2d robotAngle){

    SwerveModuleState[] wantedModuleStates = new SwerveModuleState[moduleTranslation.length];

    double factor = 1;  
    Translation2d velocityVector = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

    for(int i = 0; i < wantedModuleStates.length; i++){
      Translation2d rotationVelocity = new Translation2d(chassisSpeeds.omegaRadiansPerSecond 
        * moduleTranslation[i].getNorm(),
        moduleTranslation[i].rotateBy(Rotation2d.fromDegrees(
        90).plus(robotAngle)).getAngle());

      if(Math.abs(velocityVector.getNorm()) <= 0.1 && Math.abs(rotationVelocity.getNorm()) <= 0.1) wantedModuleStates[i] = new SwerveModuleState();
      else if(Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= 0.1) wantedModuleStates[i] = new SwerveModuleState(velocityVector.getNorm(), velocityVector.getAngle());
      else if(velocityVector.getNorm() <= 0.1) wantedModuleStates[i] = new SwerveModuleState(rotationVelocity.getNorm(), rotationVelocity.getAngle());
      

      else{
        Translation2d moduleVel = velocityVector.plus(rotationVelocity);

        if((maxVelocity / moduleVel.getNorm()) < factor) factor = maxVelocity / moduleVel.getNorm();
        
        wantedModuleStates[i] = new SwerveModuleState(moduleVel.getNorm(), moduleVel.getAngle());
      } 
      
    }
    factorVelocities(wantedModuleStates, factor);
    return wantedModuleStates;
  }
  
  private void factorVelocities(SwerveModuleState[] arr, double factor){
    for(int i = 0; i < arr.length; i++){
      arr[i].speedMetersPerSecond = arr[i].speedMetersPerSecond * factor;
    }
  }
}

