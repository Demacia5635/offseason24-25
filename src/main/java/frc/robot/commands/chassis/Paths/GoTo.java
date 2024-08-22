package frc.robot.commands.chassis.Paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.Trapezoid;

public class GoTo extends Command {

    Pose2d startPose;
    Pose2d endPose;
    Chassis chassis;
    double maxV;
    double maxA;
    Rotation2d baseDir = null;
    Trapezoid trapezoid;
    double distance = 0;
    Rotation2d rotationError = new Rotation2d();
    boolean toSpeaker;
    
    public GoTo(Pose2d target, Pose2d from, double maxV, double maxA, boolean toSpeaker, Chassis chassis) {
        this.chassis = chassis;
        this.endPose = target;
        this.startPose = from;
        this.toSpeaker = toSpeaker;
        this.maxA = maxA;
        this.maxV = maxV;
        trapezoid = new Trapezoid(maxV, maxA); 
    }

    @Override
    public void initialize() {
        if(RobotContainer.robotContainer.isRed()) {
            if(startPose != null) {
                startPose = Field.toRed(startPose);
            }
            endPose = Field.toRed(endPose);
        }
        if(startPose != null) {
            chassis.setPose(startPose);
        }   
        startPose = chassis.getPose();
        baseDir = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
    }

    @Override
    public void execute() {
        Pose2d p = chassis.getPose();
        Translation2d t = endPose.getTranslation().minus(p.getTranslation());
        double distance = t.getNorm();
        Rotation2d rotationError = t.getAngle().minus(baseDir);
        Rotation2d heading = t.getAngle().plus(rotationError);
        double v = trapezoid.calculate(distance, chassis.getMoveVelocity(), 0);
        ChassisSpeeds speed = new ChassisSpeeds(v*heading.getCos(), v*heading.getSin(), 0);
        if(toSpeaker) {
            chassis.setVelocitiesRotateToSpeaker(speed);
        } else {
            chassis.setVelocitiesRotateToAngle(speed,endPose.getRotation());
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(distance) < 10 || Math.abs(rotationError.getDegrees()) > 40;
    }

    @Override
    public void end(boolean interrupted) {
    }
    
}
