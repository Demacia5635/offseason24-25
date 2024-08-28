package frc.robot;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.ChassisConstants;
import frc.robot.chassis.commands.DriveCommand;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;
import frc.robot.utils.TalonMotor;

public class RobotContainer implements Sendable {
  public static RobotContainer robotContainer;
  private Boolean isRed = true;
  CommandXboxController commandController;
  public CommandXboxController commandController2;
  public Joystick gitar;


  public Chassis chassis;

  private LogManager LM = new LogManager();

  public Command shoot; // shoot to amp or to speaker
  public Command driveToNote;
  public Command manualIntake;
  public Command activateAmp;
  public Command disableCommand;
  public Command resetOdometry;
  public Command activatePodium;
  public Command activateShooter;
  public Command activateSubwoofer;
  

  

  public RobotContainer() {
    robotContainer = this;
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    chassis = new Chassis();
    
    commandController2 = new CommandXboxController(1);
    commandController = new CommandXboxController(0);
    chassis.setDefaultCommand(new DriveCommand(chassis, commandController));

      createCommands();
    
    SmartDashboard.putData("RobotContainer", this);
    configureBindings();
  }


  public void createCommands() {
    SmartDashboard.putNumber("VEL CALIBRATE", 0);

    SmartDashboard.putNumber("ANGLE CALIBRATE", 0);




    resetOdometry = new InstantCommand(()-> chassis.setOdometryToForward()).ignoringDisable(true);

}

  public void isRed(boolean isRed) {
    this.isRed = isRed;
  }
  public boolean isRed() {
    return isRed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is Red",this::isRed, this::isRed);
  }


  private void configureBindings() {
    commandController.b().onTrue(new RunCommand(()->chassis.setPosition()));
  
}

   
  public Command getAutonomousCommand() {
    TalonFX motor = new TalonFX(5);
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.Slot0.kP = 0.000209225899609*10*1023/122.71746031746032;
    cfg.Slot0.kS = 0.069108623637248;
    cfg.Slot0.kV = 0.00034365326824;
    cfg.Slot0.kA = 0.000702476229803;
    cfg.Feedback.SensorToMechanismRatio = (151.0 / 7.0);
    cfg.MotionMagic.MotionMagicAcceleration = 10;
    cfg.MotionMagic.MotionMagicCruiseVelocity = 100;
    cfg.MotionMagic.MotionMagicJerk = 10;
    motor.getConfigurator().apply(cfg);
    SwerveModuleState[] states = ChassisConstants.KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0,0, 1));
    SwerveModuleState state = new SwerveModuleState(1, Rotation2d.fromDegrees(90));

   
    return new RunCommand(()->motor.setControl((new MotionMagicVoltage(state.angle.getDegrees()).withSlot(0))));
  }
}
