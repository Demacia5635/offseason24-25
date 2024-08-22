package frc.robot.commands.chassis.Auto;

import static frc.robot.commands.chassis.Auto.AutoUtils.leave;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Leave extends Command {

    SequentialCommandGroup cmd;


    public Leave() {

    }


    @Override
    public void initialize() {
        cmd = new SequentialCommandGroup(leave());
    }

    
}
