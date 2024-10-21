package frc.robot.leds.strips;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Intake.Subsystem.Intake;
import frc.robot.Shooter.Commands.GoToAngle;
import frc.robot.Shooter.Subsystems.Shooter;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.leds.LedManager;
import frc.robot.leds.LedStrip;
import frc.robot.leds.LedConstants.ANGLE_CHANGER_LEDS;

public class MainLeds extends LedStrip {

  private final Intake intake;
  private final Chassis chassis;
  private final Shooter shooter;
  private Timer timer;

  public MainLeds(String name, LedManager manager, Intake intake, Chassis chassis, Shooter shooter) {
    super(name, ANGLE_CHANGER_LEDS.SIZE, manager, ANGLE_CHANGER_LEDS.OFFSET);
    this.intake = intake;
    this.chassis = chassis;
    this.shooter = shooter;
    timer = new Timer();
  }

  public void amp() {
    timer.start();
  }

  @Override
  public void periodic() {
    super.periodic();

    if (shooter.isShotoerReady && GoToAngle.isAngleReady) {
      setColor(Color.kWhite);
    } else if (intake.isNote()) {
      setColor(Color.kPurple);
    } else if (chassis.visionByNote.seeNote()) {
      setColor(Color.kGreen);
    } else {
      setColor(Color.kBlue);
    }

    if (!timer.hasElapsed(5) && timer.get() != 0) {
      setBlink(Color.kYellow);
    } else if (timer.hasElapsed(5)) {
      timer.stop();
      timer.reset();
    } 

    // switch (angleChanger.angleState) {
    //   case SPEAKER:
    //     setColor(Color.kGreen);
    //     break;
      
    //   case AMP:
    //     setColor(Color.kYellow);
    //     break;
      
    //   case TESTING:
    //     setColor(Color.kPurple);
    //     break;
      
    //   case SUBWOFFER:
    //     setColor(Color.kBlue);
    //     break;
      
    //   case DELIVERY_MID, DELIVERY_RIVAL:
    //     setColor(Color.kOrange);
    //     break;
    
    //   default:
    //     setColor(Color.kRed);
    //     break;
    // }
  }
}