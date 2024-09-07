package frc;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SysIdCmd;
import frc.robot.subsystems.ModuleS;

import com.ctre.phoenix6.hardware.TalonFX;

public class SysId {

  
    private static TalonFX motor;
    public static double velocity1;
    public static double velocity2;
    public static ModuleS module;

   /*  public double[] simpleFeedForward(double P1, double P2, double velocity1, double velocity2){ //P1 = KS + KV * V1,   P2 = KS + KV * V2 
        //(P2 - P1) = (KS + KV * 10) - (KS + KV * 5)
        // 0.1 = KV * 5
        // KV = 5/0.1 = 0.02
        // 0.1 = KS + 0.02 * 5 
        //Command myCommand = new InstantCommand(() -> mySubsystem.setMotorSpeed(1.0), mySubsystem);
        feedForwardData = new double[3];
        double deltaP = P2 - P1;
        double deltaV = velocity2 - velocity1;
        double KV = deltaV/deltaP;
        double KS = P1 - KV * velocity1;
        feedForwardData[0] = KV;
        feedForwardData[1] = KS;
        return feedForwardData;

    }
    */

    public static double[] simpleFeedForward(int id, String canbus, double power1, double power2, double Scope){
        //testRunOne(0, null, 0.1, 0.638);
        //testRunTwo(id,canbus,power2,Scope);
        runner(power1);
        velocity1 = SmartDashboard.getNumber("velocity1", 0.0);
        runner(power2);
        velocity2 = SmartDashboard.getNumber("velocity1", 0.0);
        

        double KV = getKV(power1, power2, velocity1, velocity2);
        double KS = getKS(power1, KV, velocity1);
        double[] feedForwardData = {KV,KS};
        return feedForwardData;
    }

    public static double getKV(double power1, double power2, double velocity1, double velocity2){
        return (velocity2 - velocity1)/(power2 - power1);
    }

    public static double getKS(double power, double KV, double velocity){
        return power - KV * velocity;
    }

    public static Command runner(double power){
        module = new ModuleS();
        return new SysIdCmd(module, power, 1);
    }




/*    public static Command testRunOne(int id, String canbus, double power, double Scope){
        motor = new TalonFX(id, canbus);
        return new FunctionalCommand(() -> motor.set(power), null, (inerrupted) -> {
            SmartDashboard.putNumber("velocity1 in meter/sec" ,motor.getVelocity().getValue() * Scope);
            motor.set(0);}, wait(3), motor);
    }

    public static Command testRunTwo(int id, String canbus, double power, double Scope){
        motor = new TalonFX(id, canbus);
        return new FunctionalCommand(() -> motor.set(power), null, (inerrupted) -> {
            SmartDashboard.putNumber("velocity2 in meter/sec" ,motor.getVelocity().getValue() * Scope);
            motor.set(0);}, wait(3), motor);
    }

    public static Command good(int id, double power, double scope){
        motor = new TalonFX(id);
        return new FunctionalCommand(motor.set(power), motor.getVelocity().getValue()*scope, motor.set(0), false, motor);
    }

 */

}
//5; 8.14 = 0.638