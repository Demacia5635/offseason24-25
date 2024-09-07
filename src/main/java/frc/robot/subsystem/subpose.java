package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Lengtandangle;
import frc.robot.utils.pose;

public class subpose extends SubsystemBase {
  // NetworkTable for Limelight communication
  private NetworkTable table;
  
  // Limelight data
  private double tx;
  private double ty;
  private double id;
  
  // Pose and distance calculation utilities
  private pose pose;
  private Lengtandangle dist;
  
  // Field visualization
  private Field2d field;

  // Arrays to store object data
  private String[] objects;
  private double[] dists;
  private double[] angles;

  public subpose() {
    // Initialize arrays
    objects = new String[10]; // Adjust size as needed
    dists = new double[10];
    angles = new double[10];

    // Initialize Field2d for visualization
    field = new Field2d();
    
    // Add this subsystem to SmartDashboard
    SmartDashboard.putData("RobotContainer", this);
  }

  public void initSendable(SendableBuilder builder) {
    // Add properties to be displayed on SmartDashboard
    builder.addDoubleProperty("tx", () -> tx, null);
    builder.addDoubleProperty("ty", () -> ty, null);
    builder.addDoubleProperty("id", () -> id, null);
  }

  @Override
  public void periodic() {
    // Get the Limelight NetworkTable
    table = NetworkTableInstance.getDefault().getTable("limelight");
    
    // Fetch Limelight data
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    id = table.getEntry("tid").getDouble(0);

    // Calculate distance and angle
    dist = new Lengtandangle((int)id, tx, ty);
    
    // Populate arrays with object data
    for (int i = 0; i < objects.length; i++) {
      objects[i] = dist.GetObj();
      dists[i] = dist.GetDist();
      angles[i] = dist.GetAngle();
    }
    
    // Calculate pose
    pose = new pose(objects, dists, angles);
    
    // Update field visualization
    field.setRobotPose(new Pose2d(pose.calcMyPose(), new Rotation2d()));

    // Display field on SmartDashboard
    SmartDashboard.putData("field", field);
  }
}