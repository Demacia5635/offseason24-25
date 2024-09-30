package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class ShootCommand extends Command {
    private final Intake intake;
    private double count = 0;

    public ShootCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
        SmartDashboard.putData("Shoot Notes", new InstantCommand(
            () -> new ShootCommand(intake).schedule()).withTimeout(3));
    }

    /**resets the count */
    @Override
    public void initialize() {
        intake.setBrake();
        count = 0;
    }

    @Override
    public void execute() {
        count+=0.02;
        intake.setVelocity(IntakeConstants.Parameters.SHOOT_POWER); // Set shoot velocity
        // withTimeout(IntakeConstants.Parameters.SHOOT_TIME); // Set timeout for shooting
    }

    @Override
    public boolean isFinished() {
        // Command ends when current falls under minimum
        return (intake.getMotorCurrent() < IntakeConstants.Parameters.MIN_CURRENT_TO_SHOOTER) && (count > IntakeConstants.Parameters.SHOOT_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}