package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.calc;
import frc.robot.utils.pose;

public class subpose extends SubsystemBase {
  // NetworkTable for Limelight communication
  private NetworkTable table;
  
  // Limelight data
  private double height;
  private double x_offset;
  private double y_offset;
  private double tx;
  private double ty;
  private double id;
  
  // Pose and distance calculation utilities
  private pose pose;
  private calc calc;
  
  // Field visualization
  private Field2d field;

  // Arrays to store object data
  private String objects;
  private double dists;
  private double angles;

  public subpose(double x_offset) {

    // Initialize Field2d for visualization
    field = new Field2d();
    this.x_offset = Constants.LimelightXOfset;
    // Add this subsystem to SmartDashboard
    SmartDashboard.putData(this);
  }

  public void initSendable(SendableBuilder builder) {
    // Add properties to be displayed on SmartDashboard
    
    builder.addStringProperty("name", objects != null?()->objects: ()->"r", null);
    builder.addDoubleProperty("dist", () -> dists , null);
    builder.addDoubleProperty("angle", () -> angles, null);
  }

  @Override
  public void periodic() {
    // Get the Limelight NetworkTable
    table = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    
    // Fetch Limelight data
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    id = table.getEntry("tid").getDouble(0);

    



    // Calculate distance and angle
    calc = new calc(height, tx, ty, x_offset, y_offset, id);
    //System.out.println(calc.GetDist());
    // Populate arrays with object data
      objects = calc.GetObj();
      dists = calc.GetDist();
      angles = calc.GetAngle();
    
    // Calculate pose
    pose = new pose(objects, dists, angles);
    
    // Update field visualization
    field.setRobotPose(new Pose2d(pose.calcMyPose() == null? new Translation2d(): pose.calcMyPose(), new Rotation2d()));

    // Display field on SmartDashboard
    SmartDashboard.putData("field", field);
  }
}