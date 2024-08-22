package frc.robot.commands.chassis.tests;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.ChassisConstants;
import frc.robot.subsystems.chassis.SwerveModule;

public class SetModuleAngle extends Command {

    Chassis chassis;
    Rotation2d angle = null;
    double degrees = 0;
    private GenericEntry toAngleEntry;
    private GenericEntry targetAngleEntry;
    private GenericEntry[] angleEntry = new GenericEntry[4];
    private GenericEntry[] errorEntry = new GenericEntry[4];
    ShuffleboardTab tab;
    SwerveModule[] modules;

    public 
    SetModuleAngle(Chassis chassis) {
        this.chassis = chassis;
        modules = chassis.getModules();
        addRequirements(chassis);
        tab = Shuffleboard.getTab("Module Test");
        toAngleEntry = tab.add("Set Module Angle To", 0.0).getEntry();
        targetAngleEntry = tab.add("Target Angle", 0.0).getEntry();
        for(int i = 0; i < modules.length; i++) {
            angleEntry[i] = tab.add(modules[i].name + " Angle", 0).getEntry();
            errorEntry[i] = tab.add(modules[i].name + " Error", 0).getEntry();
        }
        tab.add("Set Module Entry", this);

    }

    @Override
    public void initialize() {
        this.angle = Rotation2d.fromDegrees(toAngleEntry.getDouble(0));
        degrees = this.angle.getDegrees();
        targetAngleEntry.setDouble(angle.getDegrees());

    }

    @Override
    public void execute() {
        for(int i = 0; i < 4; i++) {
            chassis.getModule(i).setAngleByPositionPID(angle);
            double a = modules[i].getAngleDegrees();
            angleEntry[i].setDouble(a);
            errorEntry[i].setDouble(degrees - a);
        }
    }

    @Override
    public boolean isFinished() {
        var angles = chassis.getModulesAngles();
//        System.out.println(" angles = " + angles[0] + " " + angles[1] + " " + angles[2] + " " + angles[3]);
        for(double a : angles) {
            if(Math.abs(a-degrees) > ChassisConstants.MAX_STEER_ERROR) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
   
}
