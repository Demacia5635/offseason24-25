package frc.robot.leds.strips;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Shooter.Commands.Shoot;
import frc.robot.Shooter.Subsystems.Shooter;
import frc.robot.leds.LedManager;
import frc.robot.leds.LedStrip;
import frc.robot.leds.LedConstants.SHOOTER_LEDS;

public class ShooterStrip extends LedStrip {
  private final Shooter shooter;

  public ShooterStrip(String name, LedManager manager, Shooter shooter) {
    super(name, SHOOTER_LEDS.SIZE, manager, SHOOTER_LEDS.OFFSET);
    this.shooter = shooter;
  }

  @Override
  public void periodic() {
    super.periodic();

    if (shooter.isShotoerReady) {
      setColor(Color.kWhite);
    } else if (shooter.getCurrentCommand() instanceof Shoot) {
      setColor(Color.kBlue);
    } else {
      turnoff();
    }
  }
}
