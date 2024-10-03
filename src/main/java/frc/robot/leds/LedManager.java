package frc.robot.leds;

import static frc.robot.leds.LedConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.Arrays;

/**Manager of the LedStrip */
public class LedManager{

  /**all the colors for every port and every led */
  Color[][] ledColors;
  /**all the leds for every port */
  AddressableLED[] leds;
  /**all the buffer for every led for every port */
  AddressableLEDBuffer[] buffers;
  
  /**timer for blink */
  Timer timer;
  
  /**currentH for gay */
  double currentH;

  /**
   * creates Led Manager make sure to make only one and give it all to the led strip
   */
  public LedManager() {
    /*initialize all the arr of the leds */
    ledColors = new Color[STRIPS.length][];
    leds = new AddressableLED[STRIPS.length];
    buffers = new AddressableLEDBuffer[STRIPS.length];

    /*initalize all the leds for every port */
    for (int i = 0; i < STRIPS.length; i++) {
      if (STRIPS[i] != 0) {
        ledColors[i] = new Color[STRIPS[i]];
        for (int j = 0; j < STRIPS[i]; j++) {
          ledColors[i][j] = Color.kBlack;
        }
        leds[i] = new AddressableLED(i);
        buffers[i] = new AddressableLEDBuffer(STRIPS[i]);
        leds[i].setLength(STRIPS[i]);
        leds[i].start();
      }
    }

    /*initialize timer for blink */
    timer = new Timer();
    timer.start();

    /*initialize currentH for gay */
    currentH = 0;
  }

  /**
   * set a strip to be a solid color
   * @param strip the wanted strip
   * @param color the wanted color to blink
   * @return command that set the strip a certain color
   */
  public Command setColor(LedStrip strip, Color color) {
    return new RunCommand(()-> {
      for(int i = strip.offset; i < strip.size + strip.offset; i++) {
        this.ledColors[strip.port][i] = color;
      }

      update(strip.port);
    }, strip)
    .ignoringDisable(true);
  }
  
  /**
   * set a strip to be solid colors
   * @param strip the wanted strip
   * @param colors the wanted colors (notice that the size of the arr must by the size of the strip)
   * @return command that set the strip a certain color
   */
  public Command setColor(LedStrip strip, Color[] colors) {
    return new RunCommand(()-> {
      for(int i = strip.offset; i < colors.length + strip.offset; i++) {
        this.ledColors[strip.port][i] = colors[i - strip.offset];
      }

      update(strip.port);
    }, strip)
    .ignoringDisable(true);
  }
    
  /**
   * set a strip to blink in one color
   * @param strip the wanted strip
   * @param color the wanted color to blink
   * @return command that blink the strip
   */
  public Command setBlink(LedStrip strip, Color color) {
    return new RunCommand(()-> {
      for(int i = strip.offset; i < strip.size + strip.offset; i++) {
        this.ledColors[strip.port][i] = timer.get() % BLINK_TIME != 0
        ? color
        : Color.kBlack;
      }

      update(strip.port);
    }, strip)
    .ignoringDisable(true);
  }

  /**
   * set a strip to blink in certain colors
   * @param strip the wanted strip
   * @param colors the wanted colors (notice that the arr size needs to be the size of the strip)
   * @return command that blink the strip
   */
  public Command setBlink(LedStrip strip, Color[] colors) {
    return new RunCommand(()-> {
      for(int i = strip.offset; i < colors.length + strip.offset; i++) {
        this.ledColors[strip.port][i] = timer.get() % BLINK_TIME != 0 
        ? colors[i - strip.offset]
        : Color.kBlack;
      }

      update(strip.port);
    }, strip)
    .ignoringDisable(true);
  }
  
  /**
   * set the leds to being gay always
   * @param strip the wanted strip
   * @return command that makes the strip gay
   */
  public Command setSolidGay(LedStrip strip) {
    return new RunCommand(()-> {
      for (int i = strip.offset; i < strip.size + strip.offset; i++) {
        ledColors[strip.port][i] = Color.fromHSV((int) (currentH + i * 3), 255, 255);
      }

      update(strip.port);
      currentH += 3;
      currentH %= 180;

    }, strip)
    .ignoringDisable(true);
  }
  
  /**
   * set the leds to being gay blinky
   * @param strip the wanted strip
   * @return command that makes the strip gay
   */
  public Command setBlinkGay(LedStrip strip) {
    return new RunCommand(()-> {
      for (int i = strip.offset; i < strip.size + strip.offset; i++) {
        ledColors[strip.port][i] = timer.get() % BLINK_TIME != 0
        ? Color.fromHSV((int) (currentH + i * 3), 255, 255)
        : Color.kBlack;
      }

      update(strip.port);
      currentH += 3;
      currentH %= 180;

    }, strip)
    .ignoringDisable(true);
  }

  /**
   * get the colors of a strip
   * @param port the port of the strip
   * @param offset the offset of the strip
   * @param size the size of the strip
   * @return the currnet colors of the strip
   */
  public Color[] getColors(int port, int offset, int size) {
    return Arrays.copyOfRange(ledColors[port], offset, offset + size);
  }
  
  /**
   * get the colors of a strip
   * @param ledStrip the wanted strip
   * @return the current colors of the strip
   */
  public Color[] getColors(LedStrip ledStrip) {
    return getColors(ledStrip.port, ledStrip.offset, ledStrip.size);
  }
  
  /**
   * updated the certain port of leds
   * @param port the port that the leds update
   * @apiNote updated only one port at the time so the code will be faster at most cases
   */
  public void update(int port) {
    if (ledColors[port] != null) {
      for (int j = 0; j < ledColors[port].length; j++) {
        buffers[port].setLED(j, ledColors[port][j]);
      }

      leds[port].setData(buffers[port]);
    }
  }
}
