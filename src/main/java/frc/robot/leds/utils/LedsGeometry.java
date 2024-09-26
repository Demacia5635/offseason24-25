package frc.robot.leds.utils;

import java.util.Arrays;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LedsGeometry {
    private final AddressableLED[] addressableLEDs;
    private final int[] lengths;
    private final IntPair[] deFlattenedCoords;
    public final int totalLength;

    private final AddressableLEDBuffer[] buffers;
    private final boolean[] hasLMsChanged;

    /**
     * Creates a new LedsGeometry
     * 
     * @param ledStrips Pairs of ports and lengths of srtrips
     */
    public LedsGeometry(IntPair... ledStrips) {
        addressableLEDs = Arrays.stream(ledStrips).map((strip) -> new AddressableLED(strip.first))
                .toArray(AddressableLED[]::new);
        lengths = Arrays.stream(ledStrips).mapToInt((strip) -> strip.second).toArray();
        for (int i = 0; i < addressableLEDs.length; i++) {
            addressableLEDs[i].setLength(lengths[i]);
            addressableLEDs[i].start();
        }

        totalLength = Arrays.stream(lengths).sum();
        deFlattenedCoords = new IntPair[totalLength];

        int k = 0;
        for (int i = 0; i < lengths.length; i++) {
            for (int j = 0; j < lengths[i]; j++) {
                deFlattenedCoords[k + j] = new IntPair(i, j);
            }
            k += lengths[i];
        }

        buffers = Arrays.stream(lengths).mapToObj((length) -> new AddressableLEDBuffer(length))
                .toArray(AddressableLEDBuffer[]::new);

        hasLMsChanged = new boolean[buffers.length];
    }

    public void setColor(Color[] colors) {
        for (int i = 0; i < totalLength; i++) {
            IntPair indexes = deFlattenedCoords[i];
            hasLMsChanged[indexes.first] = hasLMsChanged[indexes.first]
                    || !buffers[indexes.first].getLED(indexes.second).equals(colors[i]);
            buffers[indexes.first].setLED(indexes.second, colors[i]);
        }

        for (int i = 0; i < lengths.length; i++) {
            if (hasLMsChanged[i]) {
                addressableLEDs[i].setData(buffers[i]);
            }
            
            hasLMsChanged[i] = false;
        }
    }

    public Color getColor(int index) {
        IntPair indexes = deFlattenedCoords[index];
        return buffers[indexes.first].getLED(indexes.second);
    }
}
