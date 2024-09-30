package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class IntakeConstants {

    public static class IntakeDeviceID{
        public static final int MOTOR1 = 41;
        public static final int MOTOR2 = 42;

        public static final int LIGHT_LIMIT = 0;
        public static final int MECHANICAL_LIMIT = 0;
        public static final int COUNTER_ID = 3;
      }
      public static class ConvertionParams {
        public static final double MOTOR_GEAR_RATIO = 1/3.0; // (1/number)
        public static final double MOTOR_PULSES_PER_SPIN = 2048;
      }
      public static class Parameters{
        public static final double COUNTER_M_PERA = 3790.938994;
        public static final double COUNTER_B_PERA = 65.13290974;

        public static final double DEADBAND = 0.2;
        public static final boolean IS_INVERTED = true;
        public static final boolean IS_INVERTED2 = true;

        public static final double NOT_PRESENCE_VOLTAGE = 4;

        public static final double MIN_CURRENT_TO_AMP = 50;
        public static final double MIN_CURRENT_TO_SHOOTER = 10;
        
  
  
        public static final double INTAKE_POWER = 1;
        public static final double INTAKE_POWER_SECOND = 0.8;

        public static final double INTAKE_TRANSFER_POWER = -0;
        public static final double NUM_FINAL_ROTATIONS = -0.0;
        public static final double SENSOR_TO_REST_DIST = ConvertionParams.MOTOR_PULSES_PER_SPIN/ConvertionParams.MOTOR_GEAR_RATIO*NUM_FINAL_ROTATIONS;
        public static final double INTAKE_PRE_LIMIT_POWER = 1;
  
        public static final double SHOOT_POWER = 0;
        public static final double SHOOT_TIME = 0;

        public static final double CRITICAL_CURRENT = 100;
        public static final double CRITICAL_CURRENT_WHEN_SHOOTER = 120;
  
        
        public static final double DISPENSE_POWER = -0.4;
        public static final double DISPENSE_TIME = 60;
  
  
        public static final double KP = 0.0;
        public static final double KI = 0.000;
        public static final double KD = 0;
   
        public static final double KS1 = 0.0;
        public static final double KG1 = 0.00;
        public static final double KA1 = 0.00;
        public static final double KV1 = 0.0;
      }

    
}