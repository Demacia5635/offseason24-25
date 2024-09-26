// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.leds.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.leds.SubStrip;

public class Rainbow extends Command {
    private final SubStrip strip;
    private final double speed;
    private double currentH;

    public Rainbow(SubStrip strip, double speed) {
        this.strip = strip;
        this.speed = speed;

        addRequirements(strip);
    }

    @Override
    public void initialize() {
        currentH = 0;
    }

    @Override
    public void execute() {
        Color[] colors = new Color[strip.size];
        for (int i = 0; i < colors.length; i++) {
            colors[i] = Color.fromHSV((int) (currentH + i * speed), 255, 255);
        }
        strip.setColor(colors);
        currentH += speed;
        currentH %= 180;
    }

    @Override
    public void end(boolean interrupted) {
        strip.turnOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
