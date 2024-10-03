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

}

