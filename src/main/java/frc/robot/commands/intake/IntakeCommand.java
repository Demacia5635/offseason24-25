package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.Parameters;

public class IntakeCommand extends Command {
    private final Intake intake;
    private boolean noteWasDetected = false;
    private double initialEncoderCount = 0;
    private boolean hasEntered = false;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        intake.setBrake();
        initialEncoderCount = 0;
        hasEntered = false;
        noteWasDetected = false;
        initialEncoderCount = 0;
    }

    @Override
    public void execute() {

        if (intake.isNotePresent())
            noteWasDetected = true;
        if (intake.isCriticalCurrent())
            hasEntered = true;
        if (noteWasDetected && initialEncoderCount == 0) {
            initialEncoderCount = intake.getEncoderPos();
        }

        // Check for note reaching resting spot based on encoder counts, but only after
        // initialization
        if (initialEncoderCount > 0 && intake.getEncoderPos() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST) {
            intake.setPower(0);
        } else if (noteWasDetected) { // Note detected, use transfer speed
            intake.setPower(Parameters.INTAKE_TRANSFER_POWER); // Run motors at transfer speed
        } else if (!hasEntered) {
            intake.setPower(Parameters.INTAKE_POWER, Parameters.INTAKE_POWER_SECOND); // Run motors at intake speed until note is detected
        } else {
            intake.setPower(Parameters.INTAKE_PRE_LIMIT_POWER); // Run motors at intake speed until note is detected
        }
    }

    @Override
    public boolean isFinished() {
        return initialEncoderCount > 0
                && intake.getEncoderPos() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);// Ensure motors stop
    }
}