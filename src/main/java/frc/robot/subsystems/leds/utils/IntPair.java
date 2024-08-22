package frc.robot.subsystems.leds.utils;

public class IntPair {
    public final int first;
    public final int second;

    public IntPair(int first, int second) {
        this.first = first;
        this.second = second;
    }

    @Override
    public String toString() {
        return "(%d, %d)".formatted(first, second);
    }
}
