package frc.robot.subsystem;





import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.TagPoseCalc;

public class subpose extends SubsystemBase {
  // NetworkTable for Limelight communication
  private NetworkTable table;
  
  // Limelight data
  private double x_offset;
  private double y_offset;
  private double tx;
  private double ty;
  private double id;
  private static Pigeon2 giro;

  
  // Pose and distance calculation utilities
  private TagPoseCalc Pose;
  
  // Field visualization
  private Field2d field;
  TagPoseCalc TPC;

  public subpose() {

    // Initialize Field2d for visualization
    TPC = new TagPoseCalc(tx, ty, x_offset, y_offset, id, ty);
    field = new Field2d();
    this.x_offset = Constants.LimelightXOfset;
    this.y_offset = Constants.LimelightYOfset;
    // Add this subsystem to SmartDashboard
    SmartDashboard.putData(this);

    giro = new Pigeon2(19);

  }
  public static void resetGiro(){
    giro.setYaw(0);
    System.out.println("  ");
  }
    public static void add180Giro(){
    giro.setYaw(giro.getYaw()+90);
    System.out.println("  ");
  }
      public static void addmines180Giro(){
    giro.setYaw(giro.getYaw()-90);
    System.out.println("  ");
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
    Pose = new TagPoseCalc(tx, ty, x_offset, y_offset, id, giro.getYaw());
    
    // Calculate pose
    
    // Update field visualization
    Pose2d robotPose = Pose.calculatePose();
        if (robotPose != null) {
            field.setRobotPose(robotPose);
        }

        // Display field on SmartDashboard
        // System.out.println("giro:"+giro.getYaw());

  }

  public void initSendable(SendableBuilder builder) {
    // Add properties to be displayed on SmartDashboard
    
    // builder.addStringProperty("name", this.objects != null?()->this.objects: ()->"r", null);
    // builder.addDoubleProperty("height", () -> Constants.HEIGHT_MAP.get(id) , null);
    // builder.addDoubleProperty("dist", () -> this.dists, null);
    SmartDashboard.putData("field", field);
    SmartDashboard.putData("reset", new InstantCommand(()->resetGiro()).ignoringDisable(true));
    SmartDashboard.putData("add_180", new InstantCommand(()->add180Giro()).ignoringDisable(true));
    SmartDashboard.putData("add_-180", new InstantCommand(()->addmines180Giro()).ignoringDisable(true));

  }
}