package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TestSubsystem;

public class SysIdDefault implements Sendable {

  
    public static double velocity1;
    public static double velocity2;
    private static double velocity3;
    public static double KV = 0.0;
    public static double KS = 0.0;

    private static TestSubsystem subsystem = new TestSubsystem();

    public SysIdDefault(){
        SmartDashboard.putData(this);
    }

    public static Command feedForwardRun(double power1, double power2, double power3){
        return new SequentialCommandGroup(new RunCommand(() -> {
            System.out.println("PLSSSSSS LET ME SEE THISSSS");
            subsystem.setPowers(power1);
            velocity1 = subsystem.getVel();
        },subsystem)
        .withTimeout(2.5), new RunCommand(() -> {
            subsystem.setPowers(power2);
            velocity2 = subsystem.getVel();
            KV = getKV(power1, power2, velocity1,velocity2);
            KS = getKS(power1, KV, velocity1);
        },subsystem).withTimeout(2.5), new RunCommand(()->{
            subsystem.setPowers(power3);
            velocity3 = subsystem.getVel();

        }, subsystem));

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
        double[] feedForwardData = {KV,KS};
        return feedForwardData;
    }

    public static double getKV(double power1, double power2, double velocity1, double velocity2){
        return (velocity2 - velocity1)/(power2 - power1);
    }

    public static double getKS(double power, double KV, double velocity){
        return power - KV * velocity;
    }

    public  double getKA(double power1, double power2, double power3, double velocity1, double velocity2, double velocity3, double acceleration1, double acceleration2, double acceleration3, double KV) {
        double firstEquation =  ((power2 - power1) - KV * (velocity2 - velocity1)) / (acceleration2 - acceleration1);
        double deltaP = power2-power1;
        double deltaV = velocity2 - velocity1;
        double deltaA = acceleration2 - acceleration1;
        
        
        return -1;
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