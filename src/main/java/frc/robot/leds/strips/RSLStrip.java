package frc.robot.leds.strips;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Shooter.Commands.GoToAngle;
import frc.robot.leds.LedManager;
import frc.robot.leds.LedStrip;
import frc.robot.leds.LedConstants.RSL_LEDS;

public class RSLStrip extends LedStrip {


  public RSLStrip(String name, LedManager manager) {
    super(name, RSL_LEDS.SIZE, manager, RSL_LEDS.OFFSET);
  }

  @Override
  public void periodic() {
    super.periodic();
    
    if (GoToAngle.isAngleReady) {
      setColor(Color.kWhite);
    } else {
      setColor(Color.kRed);
    }

  }
}
