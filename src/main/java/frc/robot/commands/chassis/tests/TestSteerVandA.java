package frc.robot.commands.chassis.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.SwerveModule;

public class TestSteerVandA extends Command {

    Chassis chassis;
    int n;
    double power;
    SwerveModule module;
    double lastV;
    double lastAngle;

    public TestSteerVandA(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
        module = chassis.getModule(0);
    }

    @Override
    public void initialize() {
        power = 0.2;
        n = 0;
        lastAngle = module.getAngleDegrees();
        lastV = module.getSteerVelocity();
    }

    @Override
    public void execute() {
        n++;
        double v = module.getSteerVelocity();
        double a = module.getAngleDegrees();
        // System.out.println(" last power = " + power + " v=" + v + " angle =" + a + " delat V=" + (v - lastV)
                // + " delta angle=" + (a - lastAngle));
        lastAngle = a;
        lastV = v;
        if (n == 5) {
            power = -power;
            n = 0;
        }
        module.setSteerPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        module.setSteerPower(0);
    }
}
