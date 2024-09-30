package frc.robot.commands.chassis.AutoPrevious;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.Constants.ChassisConstants;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.GoToAngleChassis;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Utils;

public class StartMiddle1 extends Command {
    double maxVel = ChassisConstants.MAX_DRIVE_VELOCITY;
    double maxAceel = ChassisConstants.DRIVE_ACCELERATION;
    Chassis chassis;
    Intake intake;
    Shooter shooter;
    boolean isRed;
    Translation2d speaker;
    SequentialCommandGroup cmd;
    
    pathPoint dummyPoint = new pathPoint(0, 0, new Rotation2d(), 0, false);
    pathPoint wingNote = offset(Field.WingNotes[1], -2,0, -4);
    pathPoint centerNote1 = offset(Field.CenterNotes[3], -1.5,-0.5,0);
    pathPoint centerNote2 = offset(Field.CenterNotes[3], -1,0,0);
    pathPoint shootPoint = offset(Field.Speaker, 2.5, 0,0);

    /** Creates a new StartTOP auto. */
    public StartMiddle1() {
        this.chassis = RobotContainer.robotContainer.chassis;
        this.intake = RobotContainer.robotContainer.intake;
        this.shooter = RobotContainer.robotContainer.shooter;
    }

    @Override
    public void initialize() {
        this.isRed = RobotContainer.robotContainer.isRed();
        speaker = Utils.speakerPosition();
        cmd = new SequentialCommandGroup(initShooter());

        addCommands(shoot());
        addCommands(takeNote());
        //addCommands(turnToSpeaker());
        addCommands(goTo(shootPoint, 0.5));
        addCommands(shoot());
        // addCommands(getNote(centerNote1));
        // addCommands(goTo(shootPoint));
        // addCommands(turnToSpeaker());
        // addCommands(shoot());

        //addCommands(getNote(centerNote2));
        //addCommands(goTo(shootPoint));
        //addCommands(turnToSpeaker());
        //addCommands(shoot());
        cmd.schedule();

    }

    @Override
    public boolean isFinished() {
        return !cmd.isScheduled();
    }

    private void addCommands(Command c) {
        cmd.addCommands(c);
    }
    

    pathPoint offset(Translation2d from, double x, double y, double angle) {
        System.out.println(" Point at " + (from.getX() + x) + " / " + (from.getY()+y));
        return new pathPoint(from.getX()+x, from.getY()+ y, Rotation2d.fromDegrees(angle),0,false);
    }


    private Command shoot() {
        return shooter.getShootCommand();
    }

    private Command initShooter() {
        return new WaitUntilCommand(shooter::getIsShootingReady);
    }

    private Command goTo(pathPoint point) {
        return goTo(point, maxVel);
    }

    private Command goTo(pathPoint point, double maxv) {
        return new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxv, maxAceel, 0, isRed);
    }

    private Command turnToSpeaker() {
        return new GoToAngleChassis(chassis, speaker);
        
    }

    private Command getNote(pathPoint point) {
        return (new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxVel, maxAceel, 1, isRed)
                .raceWith(new WaitUntilCommand(() -> Utils.seeNote())))
                .andThen(takeNote());
    }

    private Command takeNote() {
        return (new DriveToNote(chassis, 1, true).raceWith(new IntakeCommand(intake))).withTimeout(2);
    }

}