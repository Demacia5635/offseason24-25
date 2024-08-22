package frc.robot.commands.chassis.AutoPrevious;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Utils;

public class IsShooterReady extends Command {

Shooter shooter;


public IsShooterReady(Shooter shooter) {
    this.shooter = shooter;

}


    @Override
    public boolean isFinished() {
        return shooter.getIsShooting();
    }
    
}
