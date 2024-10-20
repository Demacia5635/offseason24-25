package frc.robot.leds;

import static frc.robot.leds.LedConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Arrays;

/**Manager of the LedStrip */
public class LedManager{

  /**all the colors for every led */
  Color[] ledColors;
  /**the led */
  AddressableLED led;
  /**the buffer */
  AddressableLEDBuffer buffer;
  
  /**currentH for gay */
  double currentH;

  /**
   * creates Led Manager make sure to make only one and give it all to the led strip
   */
  public LedManager() {
    ledColors = new Color[LENGTH];
    for (int i = 0; i < LENGTH; i++) {
      ledColors[i] = Color.kBlack;
    }

    led = new AddressableLED(PORT);
    buffer = new AddressableLEDBuffer(LENGTH);
    led.setLength(LENGTH);
    led.start();

    /*initialize currentH for gay */
    currentH = 0;
  }

  /**
   * set a strip to be a solid color
   * @param strip the wanted strip
   * @param color the wanted color to blink
   */
  public void setColor(LedStrip strip, Color color) {
    for(int i = strip.offset; i < strip.size + strip.offset; i++) {
      this.ledColors[i] = color;
    }

    update();
  }
  
  /**
   * set a strip to be solid colors
   * @param strip the wanted strip
   * @param colors the wanted colors (notice that the size of the arr must by the size of the strip)
   */
  public void setColor(LedStrip strip, Color[] colors) {
    for(int i = strip.offset; i < colors.length + strip.offset; i++) {
      this.ledColors[i] = colors[i - strip.offset];
    }

    update();
  }
    
  /**
   * set a strip to blink in one color
   * @param strip the wanted strip
   * @param color the wanted color to blink
   */
  public void setBlink(LedStrip strip, Color color) {
    for(int i = strip.offset; i < strip.size + strip.offset; i++) {
      this.ledColors[i] = Timer.getFPGATimestamp() % BLINK_TIME != 0
      ? color
      : Color.kBlack;
    }

    update();
  }

  /**
   * set a strip to blink in certain colors
   * @param strip the wanted strip
   * @param colors the wanted colors (notice that the arr size needs to be the size of the strip)
   */
  public void setBlink(LedStrip strip, Color[] colors) {
    for(int i = strip.offset; i < colors.length + strip.offset; i++) {
      this.ledColors[i] = Timer.getFPGATimestamp() % BLINK_TIME != 0 
      ? colors[i - strip.offset]
      : Color.kBlack;
    }

    update();
  }
  
  /**
   * set the leds to being gay always
   * @param strip the wanted strip
   */
  public void setSolidGay(LedStrip strip) {
      for (int i = strip.offset; i < strip.size + strip.offset; i++) {
        ledColors[i] = Color.fromHSV((int) (currentH + i * 3), 255, 255);
      }

      update();
      currentH += 3;
      currentH %= 180;
  }
  
  /**
   * set the leds to being gay blinky
   * @param strip the wanted strip
   */
  public void setBlinkGay(LedStrip strip) {
    for (int i = strip.offset; i < strip.size + strip.offset; i++) {
      ledColors[i] = Timer.getFPGATimestamp() % BLINK_TIME != 0
      ? Color.fromHSV((int) (currentH + i * 3), 255, 255)
      : Color.kBlack;
    }

    update();
    currentH += 3;
    currentH %= 180;
  }

  /**
   * get the colors of a strip
   * @param offset the offset of the strip
   * @param size the size of the strip
   * @return the currnet colors of the strip
   */
  public Color[] getColors(int offset, int size) {
    return Arrays.copyOfRange(ledColors, offset, offset + size);
  }
  
  /**
   * get the colors of a strip
   * @param ledStrip the wanted strip
   * @return the current colors of the strip
   */
  public Color[] getColors(LedStrip ledStrip) {
    return getColors(ledStrip.offset, ledStrip.size);
  }
  
  /**
   * updated the leds
   */
  public void update() {
    for (int i = 0; i < ledColors.length; i++) {
      buffer.setLED(i, ledColors[i]);
    }

    led.setData(buffer);
  }
}
