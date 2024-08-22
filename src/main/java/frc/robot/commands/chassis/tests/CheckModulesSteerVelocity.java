package frc.robot.commands.chassis.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class CheckModulesSteerVelocity extends Command {

    Chassis chassis;
    double velocity;


    public CheckModulesSteerVelocity(Chassis chassis, double velocity) {
        this.chassis = chassis;
        this.velocity = velocity;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
    }


    @Override
    public void execute() {
       // System.out.println("Setting velocity to " + velocity);
        for(int i = 0; i < 4; i++) {
            chassis.getModule(i).setSteerVelocity(velocity, true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }

}
