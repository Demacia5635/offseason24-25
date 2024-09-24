package frc.robot.subsystem;






import com.ctre.phoenix6.hardware.Pigeon2;

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
import frc.robot.utils.ImageProsesing.TagPoseCalc;

public class Subpose extends SubsystemBase {
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

  public Subpose() {

    // Initialize Field2d for visualization
    
    field = new Field2d();
    this.x_offset = Constants.LimelightXOfset;
    this.y_offset = Constants.LimelightYOfset;
    // Add this subsystem to SmartDashboard
    SmartDashboard.putData(this);

    giro = new Pigeon2(14);

        // Get the Limelight NetworkTable
    
    table = NetworkTableInstance.getDefault().getTable("limelight");

  }
  public static void resetGiro(){
    giro.setYaw(0);
  }
    public static void add180Giro(){
    giro.setYaw(giro.getAngle()+180);
  }
      public static void addmines180Giro(){
    giro.setYaw(giro.getAngle()-180);
  }

  

  @Override
  public void periodic() {

    
    // Fetch Limelight data
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    id = table.getEntry("tid").getDouble(0);

    

    // Calculate distance and angle
    Pose = new TagPoseCalc(tx, ty, x_offset, y_offset, id,Rotation2d.fromDegrees(giro.getAngle()));
    
    // Calculate pose
    
    // Update field visualization
    //TODO: have to fix robot rotation when he is red because he is upsidedown when he is red 
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
    SmartDashboard.putData("fieldIMGPROS", field);
    SmartDashboard.putData("resetIMG", new InstantCommand(()->resetGiro()).ignoringDisable(true));
    SmartDashboard.putData("add_180IMG", new InstantCommand(()->add180Giro()).ignoringDisable(true));
    SmartDashboard.putData("add_-180IMG", new InstantCommand(()->addmines180Giro()).ignoringDisable(true));

  }
}