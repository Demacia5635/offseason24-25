package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amp.Amp;

public class UNLock extends Command{

    Amp amp;
    double count;


    public UNLock(Amp amp) {
        this.amp = amp;
    }

    @Override
    public void initialize() {
        count = 0;
    }


    @Override
    public void execute() {
        count+=0.02;
        amp.setSnowBlowerPower(-0.1);
    }

    @Override
    public boolean isFinished() {
        return count>0.8;
    }
    
}
