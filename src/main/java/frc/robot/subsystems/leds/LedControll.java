package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.utils.Utils;

/**class that controll the leds */
public class LedControll extends SubsystemBase{

    /**the leds */
    private AddressableLED led;

    /**the buffer of the leds */
    private AddressableLEDBuffer buffer;
    
    /**the size of the  leds */
    public int size;
    
    /**used for rainbow */
    public double currentH;
    public boolean rainbow = false;

    /**
     * creates a new led controll
     * @param port the port of  the leds
     * @param count the size of the leds
     */
    public LedControll(int port, int count) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(count);
        led.setLength(buffer.getLength());
        this.size = count;
        this.currentH = 0;
        led.start();
    }
    
    /**
     * set color to the led
     * @param color the wanted color, 
     * for rgb or hex use 
     * <pre>{@code setColor(new Color("#hex")); }</pre> 
     * or 
     * <pre>{@code setColor(new Color(255, 255, 255)); }</pre>
     */
    public void setColor(Color color) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
        led.setData(buffer);
    }

    /**
     * set the color of each led
     * @param colors the wanted colors based on the index will be the led
     */
    public void setColor(Color[] colors) {
        for (int i = 0; i < colors.length; i++) {
            buffer.setLED(i, colors[i]);
        }
        led.setData(buffer);
    }

    /**
     * will set the color of all the led to rainbow
     * @apiNote needs to add {@code currentH = 0;} after the function have been calld
     */
    public void rainbow(){
        Color[] colors = new Color[size];
        for (int i = 0; i < colors.length; i++) {
            colors[i] = Color.fromHSV((int) (currentH + i * 3), 255, 255);

        }
        setColor(colors);
        currentH += 3;
        currentH %= 180;
    }

    public void turnOff(){
        setColor(Color.kBlack);
    }

    /**
     * <pre>
     * will check what color suppose to be based on the level
     * 1: if the shooter is ready = white
     * 2: if the auto drive to note is on = orange
     * 3: if there is a note in the intake = purple
     * 4: if the camera see a note = green
     * 
     * default = black
     * </pre>
     */
    @Override
    public void periodic() {
        super.periodic();

        boolean isSeeNote = Utils.seeNote();
        boolean isStart = DriveToNote.isStart;

        if(isStart){
            setColor(Color.kOrange);
        }else if (isSeeNote){
            setColor(Color.kGreen);
        } else {
            setColor(Color.kRed);
            //rainbow();
        }
    }
}