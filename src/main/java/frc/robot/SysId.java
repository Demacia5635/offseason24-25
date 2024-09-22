package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TestSubsystem;

public class SysId implements Sendable {

  
    public static double velocity1;
    public static double velocity2;
    public static double KV = 0.0;
    public static double KS = 0.0;

    private static TestSubsystem subsystem = new TestSubsystem();

    public SysId(){
        SmartDashboard.putData(this);
    }

    public static Command runs(double power1, double power2){
        return new SequentialCommandGroup(new RunCommand(() -> {
            System.out.println("PLSSSSSS LET ME SEE THISSSS");
            subsystem.setPowers(power1);
            velocity1 = subsystem.getVel();
            //SmartDashboard.putNumber("V1", velocity1);
            System.out.println(subsystem.getTrueVelocity());
            System.out.println("this is power!!! one: " + power1);
        },subsystem)
        .withTimeout(2.5), new RunCommand(() -> {
            subsystem.setPowers(power2);
            velocity2 = subsystem.getVel();
            KV = getKV(power1, power2, velocity1,velocity2);
            KS = getKS(power1, KV, velocity1);
            System.out.println("The KV is: " + KV + " The KS is: " + KS);
            System.out.println("This is the power!!!: " + power2);
            //SmartDashboard.putNumber("V2", velocity2);
        },subsystem).withTimeout(2));

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("KS", ()->KS, null);
        builder.addDoubleProperty("KV", ()->KV, null);
    }

   

    public static double getterKV(){
        return getKV(KV, KS, velocity1, velocity2);
    }

    public static double getterKS(){
        return getKS(KS, KV, velocity1);
    }
    
    public static double getVel(){
        return subsystem.getTrueVelocity();
    }

    public static double getWantedSpeed(){
        return 4;
    }

    public double getError(){
        return subsystem.getError();
    }


        
    public void calculate(double pow1, double pow2){
        double vel = subsystem.getTrueVelocity();
        double vel2 = subsystem.getTrueVelocity();
        double KV = getKV(pow1, pow2, vel, vel2);
        double KS = getKS(pow2, KV, vel2);
        System.out.println("The KS is: " + KS + " The KV is: " + KV);
    }
    


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

    public static double getKV(double power1, double power2, double velocity1, double velocity2){
        return (velocity2 - velocity1)/(power2 - power1);
    }

    public static double getKS(double power, double KV, double velocity){
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
/*
    public  Command runner(double power, int motorId, String nameForShuffleBoard){
        motor = new TalonFX(motorId, CANBUS);
        return new SysIdCmd(motor, power, nameForShuffleBoard);
    }

 */



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