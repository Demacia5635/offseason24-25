package frc.robot.commands.chassis.Auto;

import static frc.robot.commands.chassis.Auto.AutoUtils.speaker;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.chassis.Chassis;

public class IsChassisInAngle extends Command {

    Chassis chassis;

    public IsChassisInAngle(Chassis chassis) {
        this.chassis = chassis;
        
    }

    @Override
    public boolean isFinished() {
        Rotation2d angleOdometry = AutoUtils.speaker.minus(chassis.getPose().getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180)); 
        Rotation2d angleAprilTag = RobotContainer.robotContainer.vision.getSpeakerAngle().minus(chassis.getAngle());
        double dif = (chassis.distanceFromSpeaker() > 2.5) ? angleOdometry.getDegrees()
        : angleAprilTag.getDegrees();

        System.out.println(dif);
        return Math.abs(dif) < 4;
    }
    
}
