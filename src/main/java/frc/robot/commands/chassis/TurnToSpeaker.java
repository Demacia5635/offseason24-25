// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.shooter.ShooterConstants;

public class TurnToSpeaker extends Command {
    
    Chassis chassis;
    Pose2d speaker;
    double wantedAngle = 0;
    PIDController pid = new PIDController(0.31, 0.006, 0.0000025);

    /** Creates a new TurnToSpeaker. */
    public TurnToSpeaker(Chassis chassis) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.chassis = chassis;
        speaker = DriverStation.getAlliance().get() == Alliance.Red ? ShooterConstants.Speaker.RED_ALLIANCE_SPEAKER : ShooterConstants.Speaker.BLUE_ALLIANCE_SPEAKER ;
        addRequirements(chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        wantedAngle = Math.atan((speaker.getX() - chassis.getPose().getX()) / (speaker.getY() - chassis.getPose().getY()));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ChassisSpeeds speeds;
        if (wantedAngle > chassis.getAngle().getDegrees()){
            speeds = new ChassisSpeeds(0, 0, pid.calculate(chassis.getAngle().getDegrees(), wantedAngle) * 180);
        } else {
            speeds = new ChassisSpeeds(0, 0, -1*(pid.calculate(chassis.getAngle().getDegrees(), wantedAngle) * 180));
        }
            chassis.setVelocities(speeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(chassis.getAngle().getDegrees() - wantedAngle) <= 1;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("angle", ()-> wantedAngle, null);
    }
}
