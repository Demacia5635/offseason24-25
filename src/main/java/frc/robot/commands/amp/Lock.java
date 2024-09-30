package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amp.Amp;

public class Lock extends Command{

    Amp amp;


    public Lock(Amp amp) {
        this.amp = amp;
    }


    @Override
    public void execute() {
        amp.setSnowBlowerPower(0.1);
    }

    @Override
    public boolean isFinished() {
        return amp.getNeedStop();
    }
    
}
