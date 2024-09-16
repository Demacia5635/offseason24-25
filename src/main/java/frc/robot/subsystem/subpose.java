package frc.robot.subsystem;




import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.utils.calc;
import frc.robot.utils.pose;

public class subpose extends SubsystemBase {
  // NetworkTable for Limelight communication
  private NetworkTable table;
  
  // Limelight data
  private double x_offset;
  private double y_offset;
  private double tx;
  private double ty;
  private double id;
  
  private Pigeon2 giro;
  
  // Pose and distance calculation utilities
  private pose pose;
  private calc calc;
  
  // Field visualization
  private Field2d field;

  // Arrays to store object data
  private String objects;
  private double dists;
  private double angles;
  public subpose() {

    // Initialize Field2d for visualization
    field = new Field2d();
    this.x_offset = Constants.LimelightXOfset;
    this.y_offset = Constants.LimelightYOfset;
    // Add this subsystem to SmartDashboard
    SmartDashboard.putData(this);

    giro = new Pigeon2(14);

  }
  public static void resetGiro(Pigeon2 giro){
    giro.reset();
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
    calc = new calc(tx, ty, x_offset, y_offset, id);
    //System.out.println(calc.GetDist());
    // Populate arrays with object data
    this.objects = calc.GetObj();
    this.dists = calc.GetDist();
    this.angles = calc.GetAngle();
    
    // Calculate pose
    pose = new pose(objects, dists, angles, giro.getAngle());
    
    // Update field visualization
    Pose2d robotPose = pose.calcMyPose();
        if (robotPose != null) {
            field.setRobotPose(robotPose);
        }

        // Display field on SmartDashboard
        SmartDashboard.putData("field", field);

  }

  public void initSendable(SendableBuilder builder) {
    // Add properties to be displayed on SmartDashboard
    
    builder.addStringProperty("name", this.objects != null?()->this.objects: ()->"r", null);
    builder.addDoubleProperty("height", () -> Constants.HEIGHT_MAP.get(id) , null);
    builder.addDoubleProperty("dist", () -> this.dists, null);

  }
}