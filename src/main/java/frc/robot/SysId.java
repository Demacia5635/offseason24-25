package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.TestSubsystem;

public class SysId implements SendableBuilder {

  
    public double velocity1;
    public double velocity2;
    public double KV;
    public double KS;

    private TestSubsystem subsystem = new TestSubsystem();

    public SysId(){}

    public Command runs(double power1, double power2){
        return new ParallelCommandGroup(new InstantCommand(() -> {
            subsystem.setPowers(power1);
            velocity1 = subsystem.getTrueVelocity();
            System.out.println(subsystem.getTrueVelocity());
        },subsystem)
        .withTimeout(2.5), new InstantCommand(() -> {
            subsystem.setPowers(power2);
            velocity2 = subsystem.getTrueVelocity();
            KV = getKV(power1, power2, velocity1,velocity2);
            KS = getKS(power1, KV, velocity1);
            System.out.println("The KV is: " + KV + " The KS is: " + KS);
            SmartDashboard.putNumber("KS", KS);
            SmartDashboard.putNumber("KV", KV);
        },subsystem));

    }
    
    public double getVel(){
        return subsystem.getTrueVelocity();
    }

    public double getWantedSpeed(){
        return 4;
    }

    public double getError(){
        return subsystem.getError();
    }


    /*    
    public void calculate(double pow1, double pow2){
        runs(pow1);
        double vel = subsystem.getTrueVelocity();
        runs(pow2);
        double vel2 = subsystem.getTrueVelocity();
        double KV = getKV(pow1, pow2, vel, vel2);
        double KS = getKS(pow2, KV, vel2);
        System.out.println("The KS is: " + KS + " The KV is: " + KV);
    }
    */


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

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }

    @Override
    public void setSmartDashboardType(String type) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSmartDashboardType'");
    }

    @Override
    public void setActuator(boolean value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setActuator'");
    }

    @Override
    public void setSafeState(Runnable func) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSafeState'");
    }

    @Override
    public void addBooleanProperty(String key, BooleanSupplier getter, BooleanConsumer setter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addBooleanProperty'");
    }

    @Override
    public void publishConstBoolean(String key, boolean value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'publishConstBoolean'");
    }

    @Override
    public void addIntegerProperty(String key, LongSupplier getter, LongConsumer setter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addIntegerProperty'");
    }

    @Override
    public void publishConstInteger(String key, long value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'publishConstInteger'");
    }

    @Override
    public void addFloatProperty(String key, FloatSupplier getter, FloatConsumer setter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addFloatProperty'");
    }

    @Override
    public void publishConstFloat(String key, float value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'publishConstFloat'");
    }

    @Override
    public void addDoubleProperty(String key, DoubleSupplier getter, DoubleConsumer setter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addDoubleProperty'");
    }

    @Override
    public void publishConstDouble(String key, double value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'publishConstDouble'");
    }

    @Override
    public void addStringProperty(String key, Supplier<String> getter, Consumer<String> setter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addStringProperty'");
    }

    @Override
    public void publishConstString(String key, String value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'publishConstString'");
    }

    @Override
    public void addBooleanArrayProperty(String key, Supplier<boolean[]> getter, Consumer<boolean[]> setter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addBooleanArrayProperty'");
    }

    @Override
    public void publishConstBooleanArray(String key, boolean[] value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'publishConstBooleanArray'");
    }

    @Override
    public void addIntegerArrayProperty(String key, Supplier<long[]> getter, Consumer<long[]> setter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addIntegerArrayProperty'");
    }

    @Override
    public void publishConstIntegerArray(String key, long[] value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'publishConstIntegerArray'");
    }

    @Override
    public void addFloatArrayProperty(String key, Supplier<float[]> getter, Consumer<float[]> setter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addFloatArrayProperty'");
    }

    @Override
    public void publishConstFloatArray(String key, float[] value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'publishConstFloatArray'");
    }

    @Override
    public void addDoubleArrayProperty(String key, Supplier<double[]> getter, Consumer<double[]> setter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addDoubleArrayProperty'");
    }

    @Override
    public void publishConstDoubleArray(String key, double[] value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'publishConstDoubleArray'");
    }

    @Override
    public void addStringArrayProperty(String key, Supplier<String[]> getter, Consumer<String[]> setter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addStringArrayProperty'");
    }

    @Override
    public void publishConstStringArray(String key, String[] value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'publishConstStringArray'");
    }

    @Override
    public void addRawProperty(String key, String typeString, Supplier<byte[]> getter, Consumer<byte[]> setter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addRawProperty'");
    }

    @Override
    public void publishConstRaw(String key, String typeString, byte[] value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'publishConstRaw'");
    }

    @Override
    public BackendKind getBackendKind() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getBackendKind'");
    }

    @Override
    public boolean isPublished() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isPublished'");
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }

    @Override
    public void clearProperties() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'clearProperties'");
    }

    @Override
    public void addCloseable(AutoCloseable closeable) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addCloseable'");
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