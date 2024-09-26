// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.leds.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.leds.LedConstants;
import frc.robot.leds.SubStrip;
import frc.robot.leds.utils.IndividualLed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LedBlink extends SequentialCommandGroup {

    public LedBlink(SubStrip led, IndividualLed... color) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new InstantCommand(()-> led.turnOff() ,led),
            new WaitCommand(LedConstants.BLINK_WAIT_TIME / 1000),
            new InstantCommand(()-> led.setColor(color), led),
            new WaitCommand(LedConstants.BLINK_WAIT_TIME / 1000)
            );
            
        addRequirements(led);
    }
        
    /** Creates a new Blink. */
    public LedBlink(SubStrip led, Color color) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
      
        addCommands(
            new InstantCommand(()-> led.turnOff(),led) ,
            new WaitCommand(LedConstants.BLINK_WAIT_TIME / 1000),
            new InstantCommand(()-> led.setColor(color), led),
            new WaitCommand(LedConstants.BLINK_WAIT_TIME / 1000)
        );

        addRequirements(led);
    }

    public LedBlink(SubStrip led, Color[] color) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            new InstantCommand(()-> led.turnOff(), led),
            new WaitCommand(LedConstants.BLINK_WAIT_TIME / 1000),
            new InstantCommand(()-> led.setColor(color)),
            new WaitCommand(LedConstants.BLINK_WAIT_TIME / 1000)
        );

        addRequirements(led);
    }
}
