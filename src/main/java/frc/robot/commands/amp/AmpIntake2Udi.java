// package frc.robot.commands.amp;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.amp.AmpUdi;
// import static frc.robot.subsystems.amp.AmpConstantsUdi.Parameters.*;

// public class AmpIntake2Udi extends Command {

//     private final AmpUdi amp;
//     private boolean noteWasDetected = false;
//     private double initialEncoderCount = 0;
//     private boolean hasEntered = false;

//     private boolean debug = false;

//     public AmpIntake2Udi(AmpUdi amp) {
//         this.amp = amp;
//         addRequirements(amp);

//     }

//     @Override
//     public void initialize() {
//         super.initialize();
//         amp.setArmBrake();
//         amp.setWheelsBrake();
//         amp.wheelsEncoderReset();
//         amp.setArmTargetAngle(SENSEOR_ANGLE);
//         initialEncoderCount = 0;
//         noteWasDetected = false;
//         hasEntered = false;
//     }

//     private void debug(String msg) {
//         if (debug)
//             System.out.println(msg);
//     }

//     @Override
//     public void execute() {

//         if (!noteWasDetected && amp.isSensingNote()) {
//             noteWasDetected = true;
//             initialEncoderCount = amp.getSmallWheelsPosition();
//         }
//         if (amp.isIntakePushingNote()) {
//             hasEntered = true;
//         }

//         // set power based on note position - at target, after sensor, between wheels
//         // and sensor or before wheels
//         if (noteWasDetected) {
//             if (atPosition()) {
//                 amp.setWheelsPower(0); // in place
//             } else {
//                 amp.setWheelsPower(INTAKE_POST_SENSOR_POWER); // Run motors after sensor to target based on encoder
//             }
//         } else if (!hasEntered) { // didn't reach the small wheels yet
//             amp.setWheelsPower(INTAKE_INITIAL_POWER);
//         } else { // reached small wheels but not the sensor
//             amp.setWheelsPower(INTAKE_PRE_SENSOR_POWER);
//         }
//     }

//     private boolean atPosition() {
//         return noteWasDetected && amp.getSmallWheelsPosition() >= initialEncoderCount + SENSOR_TO_REST_DIST;
//     }

//     @Override
//     public boolean isFinished() {
//         // Command ends when note is detected and reaches resting spot
//         return atPosition();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         amp.setWheelsPower(0);// Ensure motors stop
//     }

// }

