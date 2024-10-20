package frc.robot.leds.strips;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.leds.LedManager;
import frc.robot.leds.LedStrip;
import frc.robot.leds.LedConstants.ANGLE_CHANGER_LEDS;

public class AngleChangerStrip extends LedStrip {

  private final AngleChanger angleChanger;

  public AngleChangerStrip(String name, LedManager manager, AngleChanger angleChanger) {
    super(name, ANGLE_CHANGER_LEDS.SIZE, manager, ANGLE_CHANGER_LEDS.OFFSET);
    this.angleChanger = angleChanger;
  }

  @Override
  public void periodic() {
    super.periodic();
    
    switch (angleChanger.angleState) {
      case SPEAKER:
        setColor(Color.kGreen);
        break;
      
      case AMP:
        setColor(Color.kYellow);
        break;
      
      case TESTING:
        setColor(Color.kPurple);
        break;
      
      case SUBWOFFER:
        setColor(Color.kBlue);
        break;
      
      case DELIVERY_MID, DELIVERY_RIVAL:
        setColor(Color.kOrange);
        break;
    
      default:
        setColor(Color.kRed);
        break;
    }
  }
}
