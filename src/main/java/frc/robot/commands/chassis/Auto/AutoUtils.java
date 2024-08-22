package frc.robot.commands.chassis.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ChassisConstants;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.GoToAngleChassis;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Utils;
import frc.robot.subsystems.chassis.ChassisConstants.*;

public class AutoUtils {

    private static double offset = 10;

    static Shooter shooter = RobotContainer.robotContainer.shooter;
    static Chassis chassis = RobotContainer.robotContainer.chassis;
    static Intake intake = RobotContainer.robotContainer.intake;
    static double maxVel = ChassisConstants.MAX_DRIVE_VELOCITY;
    static double maxAceel = ChassisConstants.DRIVE_ACCELERATION;
    static pathPoint dummyPoint = new pathPoint(0, 0, new Rotation2d(), 0, false);
    static Translation2d speaker = Utils.speakerPosition();
    static boolean isRed = RobotContainer.robotContainer.isRed();


    public static void addCommands(Command c, SequentialCommandGroup cmd) {
        cmd.addCommands(c);
    }

    public static boolean isInside(Translation2d wantedPos){
        return Math.abs(wantedPos.getX() - chassis.getPoseX()) <=  offset &&
        Math.abs(wantedPos.getY() - chassis.getPoseY()) <= offset;
    }
    
    public static pathPoint offset(Translation2d from, double x, double y, double angle) {
        return offset(from, x, y, angle,0);
    }

    public static pathPoint offset(Translation2d from, double x, double y, double angle, double radius) {
        return new pathPoint(from.getX()+x, from.getY()+ y, Rotation2d.fromDegrees(angle),radius,false);
    }


    public static  Command shoot(double delay) {
        return shooter.getShootCommand().andThen(new WaitCommand(delay));
    }

    public static  Command initShooter() {
        return new WaitUntilCommand(() -> shooter.getIsShootingReady());
    }

    public static  Command goTo(pathPoint point) {
        return goTo(point, maxVel, true);
    }
    public static  Command goTo(pathPoint point, double maxV) {
        return goTo(point, maxV, true);
    }
    public static  Command goToRotate(pathPoint point, double maxV, double rate) {
        return new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxV, maxAceel,
         0, false).setAutoRotate(rate);
    }

    public static Command goToMultiple(pathPoint[] points, double maxVel){
        return new PathFollow(chassis, points, maxVel, maxAceel, 0, false);
    }
    public static  Command goTo(pathPoint point, double maxv, boolean toSpeaker) {
        return new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxv, maxAceel, 0, toSpeaker);
    }
    public static  Command goTo(pathPoint point, double maxv, boolean toSpeaker, double endV) {
        return new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxv, maxAceel, endV, toSpeaker);
    }


    public static  Command getNote(pathPoint point) {
        return (new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxVel, maxAceel, 1, false)
                .raceWith(new WaitUntilCommand(() -> Utils.seeNote())))
                .andThen(takeNote());
    }

    public static  Command takeNote() {
        return new DriveToNote(chassis, 1.5, true)
        .raceWith(new IntakeCommand(intake)).andThen(new IntakeCommand(intake)).withTimeout(2);
    }
    public static Command isReadyToShoot(){
        return new WaitUntilCommand(()->shooterReady());
    }

    public static boolean shooterReady() {
        return shooter.getIsShootingReady() && Math.abs(chassis.getErrorSpeakerAngle()) < 4;
    }

    public static Command turnToSpeakerAndWaitForReady() {
        return new RunCommand(()->chassis.setVelocitiesRotateToSpeaker(new ChassisSpeeds()), chassis)
            .raceWith(isReadyToShoot());
    }

    public static Command leave() {
        return new RunCommand(()-> chassis.setVelocities(
            new ChassisSpeeds(1.5, 0, 0)), chassis).withTimeout(3);
    }

    
}
