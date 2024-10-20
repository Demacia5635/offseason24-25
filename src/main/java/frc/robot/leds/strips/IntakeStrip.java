package frc.robot.leds.strips;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Intake.Subsystem.Intake;
import frc.robot.leds.LedManager;
import frc.robot.leds.LedStrip;
import frc.robot.leds.LedConstants.INTAKE_LEDS;

public class IntakeStrip extends LedStrip {
  private final Intake intake;
  
  public IntakeStrip(String name, LedManager manager, Intake intake) {
    super(name, INTAKE_LEDS.SIZE, manager, INTAKE_LEDS.OFFSET);
    this.intake = intake;
  }

  @Override
  public void periodic() {
    super.periodic();
    if (intake.isNote()) {
      setColor(Color.kPurple);
    } else {
      turnoff();
    }
  }
}
