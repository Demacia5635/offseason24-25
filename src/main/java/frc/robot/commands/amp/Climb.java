package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amp.Amp;

public class Climb extends Command {

    Amp amp;
    
    public Climb(Amp amp) {
        this.amp = amp;
    }

    @Override
    public void initialize() {
        amp.unlock();
    }

    @Override
    public void execute() {
        amp.setArmPower(.5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    
}
