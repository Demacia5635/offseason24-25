package frc.robot.leds.strips;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.leds.LedManager;
import frc.robot.leds.LedStrip;
import frc.robot.leds.LedConstants.RSL_LEDS;

public class RSLStrip extends LedStrip {

  private final Chassis chassis;

  public RSLStrip(String name, LedManager manager, Chassis chassis) {
    super(name, RSL_LEDS.SIZE, manager, RSL_LEDS.OFFSET);
    this.chassis = chassis;
  }

  @Override
  public void periodic() {
    super.periodic();
    
    if (chassis.isAutoIntake()) {
      setColor(Color.kDarkGreen);
    } else {
      setColor(Color.kFirstRed);
    }
  }
}
