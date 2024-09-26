package frc.robot.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.leds.commands.LedBlink;
import frc.robot.leds.utils.IndividualLed;

public class SubStrip extends SubsystemBase {
    public final int size;
    private final int offset;

    public SubStrip(int offset, int size) {
        this.offset = offset;
        this.size = size;
    }

    public SubStrip(int size){
        this.offset = 0;
        this.size = size;
    }

    public void setColor(IndividualLed... leds) {
        // LedsManager.getInstance().update(Arrays.stream(leds)
        //         .map((led) -> new IndividualLed(offset + led.index, led.color)).toArray(IndividualLed[]::new));
        for (IndividualLed led : leds) {
            led.index += offset;
        }
        LedsManager.getInstance().update(leds);
    }

    public void setColor(Color color) {
        // setColor(IntStream.range(0, size).mapToObj((i) -> new IndividualLed(i, color))
        //         .toArray(IndividualLed[]::new));
        IndividualLed[] leds = new IndividualLed[size];
        for (int i = 0; i < size; i++) {
            leds[i] = new IndividualLed(i + offset, color);
        }
        LedsManager.getInstance().update(leds);
    }

    public void turnOff() {
        // setColor(IntStream.range(0, size).mapToObj((i) -> new IndividualLed(i, Color.kBlack))
        //         .toArray(IndividualLed[]::new));
        setColor(Color.kBlack);
    }

    public Color[] getColors() {
        return LedsManager.getInstance().getColors(offset, size);
    }

    public void setColor(Color[] colors) {
        // setColor(IntStream.range(0, Math.min(colors.length, size)).mapToObj((i) -> new IndividualLed(i, colors[i]))
        //         .toArray(IndividualLed[]::new));
        int length = Math.min(colors.length, size);
        IndividualLed[] leds = new IndividualLed[length];
        for (int i = 0; i < length; i++) {
            leds[i] = new IndividualLed(i + offset, colors[i]);
        }
        LedsManager.getInstance().update(leds);
    }
    
    public void setBlink(IndividualLed... leds) {
        new LedBlink(this, leds).repeatedly().schedule();
    }

    public void setBlink(Color color){
        new LedBlink(this, color).repeatedly().schedule();
    }

    public void setBlink(Color[] colors) {
        new LedBlink(this, colors).repeatedly().schedule();
    }

    @Override
    public void periodic() {

    }
}
