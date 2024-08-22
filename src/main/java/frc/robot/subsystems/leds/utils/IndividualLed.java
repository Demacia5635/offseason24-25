package frc.robot.subsystems.leds.utils;

import edu.wpi.first.wpilibj.util.Color;

public class IndividualLed {
    public int index;
    public final Color color;

    public IndividualLed(int index, Color color) {
        this.index = index;
        this.color = color;
    }
}
