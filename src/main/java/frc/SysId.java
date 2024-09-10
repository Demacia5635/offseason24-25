package frc;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SysIdCmd;

import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.*;

public class SysId {

  
    private  TalonFX motor;
    public  double velocity1;
    public  double velocity2;

  

    public  double[] simpleFeedForward(double power1, double power2, double Scope){
        //testRunOne(0, null, 0.1, 0.638);
        //testRunTwo(id,canbus,power2,Scope);
        velocity1 = SmartDashboard.getNumber("v1", 0.0);
        velocity2 = SmartDashboard.getNumber("v2", 0.0);
        

        double KV = getKV(power1, power2, velocity1, velocity2);
        double KS = getKS(power1, KV, velocity1);
        double KA = getKA(power1, power2, power1, power2, Scope, KS, KV);
        double[] feedForwardData = {KV,KS,KA};
        return feedForwardData;
    }

    public  double getKV(double power1, double power2, double velocity1, double velocity2){
        return (velocity2 - velocity1)/(power2 - power1);
    }

    public  double getKS(double power, double KV, double velocity){
        return power - KV * velocity;
    }

    public  double getKA(double power1, double power2, double velocity1, double velocity2, double acceleration1, double acceleration2, double KV) {
        return ((power2 - power1) - KV * (velocity2 - velocity1)) / (acceleration2 - acceleration1);
    }

    public void showFeedForward(double power1, double power2, double Scope){
        velocity1 = SmartDashboard.getNumber("v1", 0.0);
        velocity2 = SmartDashboard.getNumber("v2", 0.0);

        double KV = getKV(power1,power2,velocity1,velocity2);
        double KS = getKS(power1, KV, velocity1);
        //double KA = getKA(power1, power2,velocity1,velocity2)
        SmartDashboard.putNumber("KV", KV);
        SmartDashboard.putNumber("KS", KS);
    }
    
        //p1 = ks * Math.signum(velocity1) + kv * velocity1 + ka * acceleration1;
        //p2 = ks * Math.signum(velocity2) + kv * velocity2 + ka * acceleration2
        /*
         * 0.1 = 2kv 
         * 
         */

    public  Command runner(double power, int motorId, String nameForShuffleBoard){
        motor = new TalonFX(motorId, CANBUS);
        return new SysIdCmd(motor, power, nameForShuffleBoard);
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