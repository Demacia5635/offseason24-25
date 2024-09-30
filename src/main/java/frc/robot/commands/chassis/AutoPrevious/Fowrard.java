package frc.robot.commands.chassis.AutoPrevious;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Utils;

public class Fowrard extends Command {
    double maxVel = ChassisConstants.MAX_DRIVE_VELOCITY;
    double maxAceel = ChassisConstants.DRIVE_ACCELERATION;
    Chassis chassis;
    Intake intake;
    Shooter shooter;
    boolean isRed;
    Translation2d speaker;
    SequentialCommandGroup cmd;
    double direction;


    /** Creates a new StartTOP auto. */
    public Fowrard() {
        this.chassis = RobotContainer.robotContainer.chassis;
        this.intake = RobotContainer.robotContainer.intake;
        this.shooter = RobotContainer.robotContainer.shooter;
    }

    @Override
    public void initialize() {
        this.isRed = RobotContainer.robotContainer.isRed();
        speaker = Utils.speakerPosition();
        cmd = new SequentialCommandGroup(initShooter());
        direction = isRed ? 1 : -1;

        addCommands(new RunCommand( ()-> chassis.setVelocities(new ChassisSpeeds(2 * direction,0,0))));
        cmd.schedule();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private void addCommands(Command c) {
        cmd.addCommands(c);
    }
    


    private Command shoot() {
        return shooter.getShootCommand();
    }

    private Command initShooter() {
        return new WaitUntilCommand(() -> shooter.getIsShootingReady());
    }


}