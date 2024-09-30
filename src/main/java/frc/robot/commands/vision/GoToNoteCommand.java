package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class GoToNoteCommand extends Command {
 private final Chassis chassis;
 private double Angle;
 private double Dist;
 private double Dist_OfSet;
 private double No_Dist_OfSet;
 private double Mol;
 private double[] Array;
 private double[] llpython;

 // Constructor initializes the chassis and adds it as a requirement
 public GoToNoteCommand(Chassis chassis, double Dist_OfSet, double No_Dist_OfSet) {
    this.chassis = chassis;
    this.Dist_OfSet = Dist_OfSet;
    this.No_Dist_OfSet = No_Dist_OfSet;
    addRequirements(chassis);
 }

 @Override
 public void initialize() {
    // Stop the chassis at the start of the command
    chassis.stop();
 }

 @Override
 public void execute() {
    // Get the distance and angle from the GetDistAndAngle method
    Array = GetDistAndAngle( Dist_OfSet, No_Dist_OfSet);
    Dist = Array[0];
    Angle = Array[1];
    // Create a new ChassisSpeeds object with the calculated angles and distances
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0.7*Math.signum(Angle));
    // Set the velocities of the chassis
    chassis.setVelocities(speeds);
 }
 
 @Override
 public boolean isFinished() {
   // The command finishes when the absolute value of the angle is less than or equal to 2
   return Math.abs(Angle) <= 2;
 }

 @Override
 public void end(boolean interrupted) {
   // Stop the chassis when the command ends
   chassis.stop();
 }

 // Method to get the distance and angle from the Limelight
 public double[] GetDistAndAngle(double Dist_OfSet, double No_Dist_OfSet){
   double[] Arr = new double[2];
   // Get the data from the Limelight NetworkTable
   llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
   // Extract the angle and distance from the Limelight data
   Angle = llpython[1];
   Dist = llpython[0];
   // Calculate the slope of the line to the target
   Mol = Dist * Math.tan(Angle);
   // Calculate the difference between the expected and actual distances
   Mol = Math.abs(No_Dist_OfSet - Mol);
   // Adjust the distance for the offset
   Dist = Dist + Dist_OfSet;
   // Calculate the angle to the target
   Angle = Math.atan(Mol/Dist);
   // Store the distance and angle in an array
   Arr[0] = Dist;
   Arr[1] = Angle;
   return Arr;
 }
}
