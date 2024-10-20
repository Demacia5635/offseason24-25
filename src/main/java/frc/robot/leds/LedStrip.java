package frc.robot.leds;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**Strip of led */
public class LedStrip extends SubsystemBase{
  /**the size of the strip */
  public int size;

  /**the offset the strip have on the port */
  public int offset;
  
  /**the manager of the leds */
  public LedManager manager;

  /**
   * creates a new strip
   * @param name the name of the strip (for network table)
   * @param size the size of the strip
   * @param ledManager the led manager 
   * @param offset the offset of the strip in the port
   */
  public LedStrip(String name, int size, LedManager ledManager, int offset) {
    /*initialize pera */
    this.size = size;
    this.offset = offset;
    this.manager = ledManager;
    
    /*puts on the network table all the colors */
    setName(name);
    SmartDashboard.putData(name, this);
  }

  /**
   * creates a new strip
   * @param name the name of the strip (for network table)
   * @param size the size of the strip
   * @param ledManager the led manager 
   */
  public LedStrip(String name, int size, LedManager ledManager) {
    /*set offset to 0 */
    this.offset = 0;

    /*initialize pera */
    this.size = size;
    this.manager = ledManager;
    
    /*puts on the network table all the colors */
    setName(name);
    SmartDashboard.putData(name, this);
  }

  /**
   * set the strip to one color
   * @param color the wanted color
   * @return command that makes the strip one solid color
   */
  public Command setColor(Color color) {
    return manager.setColor(this, color);
  }
  
  /**
   * set the strip to solid colors
   * @param colors the wanted colors
   * @return command that makes the strip solid colors
   */
  public Command setColor(Color[] colors) {
    return manager.setColor(this, colors);
  }

  /**
   * set blink to strip
   * @param color the wanted color
   * @return command that makes the strip blink with one color
   */
  public Command setBlink(Color color) {
    return manager.setBlink(this, color);
  }

  /**
   * set blink to strip
   * @param colors the wanted colors
   * @return command that makes the strip blink with diffrent colors
   */
  public Command setBlink(Color[] colors) {
    return manager.setBlink(this, colors);
  }

  /**
   * turn the strip off
   * @return command that turn off the leds
   */
  public Command turnoff() {
    return setColor(Color.kBlack);
  }
  
  /**
   * set the strip solid gay
   * @return command that make the strip gay
   */
  public Command setSolidGay() {
    return manager.setSolidGay(this);
  }

  /**
   * set the strip blink gay
   * @return command that make the strip gay blink
   */
  public Command setBlinkGay() {
    return manager.setBlinkGay(this);
  }

  /**
   * get the colors of the strip
   * @return arr of colors that are the colors of the leds in the leds manager
   */
  public Color[] getColors() {
    return manager.getColors(this);
  }

  /**
   * get the colors as a hex code
   * {@code String.format("#%02X%02X%02X", (int) (red * 255), (int) (green * 255), (int) (blue * 255))}
   * @return arr of string as hex code 
   */
  public String[] getColorsAsHex() {
    Color[] colors = getColors();
    String[] ans = new String[colors.length];
    for(int i = 0; i < colors.length; i++) {
      ans[i] = colors[i].toHexString();
    }
    
    return ans;
  }

  /**
   * put the colors in network table
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    
    builder.addStringArrayProperty("Colors", this::getColorsAsHex, null);
  }
}
