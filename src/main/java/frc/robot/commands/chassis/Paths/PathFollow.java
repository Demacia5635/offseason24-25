package frc.robot.commands.chassis.Paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.subsystems.chassis.ChassisConstants.*;
import edu.wpi.first.math.trajectory.Trajectory.State;

import java.util.ArrayList;
import java.util.List;

import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.Leg;
import frc.robot.PathFollow.Util.RoundedPoint;
import frc.robot.PathFollow.Util.Segment;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.chassis.*;
import frc.robot.utils.TrapezoidNoam;

public class PathFollow extends Command {
  Timer timer = new Timer();

  Chassis chassis;
  RoundedPoint[] corners;
  Pose2d closestAprilTag = new Pose2d();

  Pose2d chassisPose = new Pose2d();
  double distanceOffset = 0.01;
  double pathLength;

  double totalLeft;
  int segmentIndex = 0;

  Segment[] segments;
  Translation2d vecVel;
  Rotation2d wantedAngle;

  TrapezoidNoam driveTrapezoid;
  TrapezoidNoam rotationTrapezoid;
  Field2d trajField = new Field2d();

  double driveVelocity = 0;
  double rotationVelocity = 0;
  static double fieldLength = 16.54; // in meters
  static double fieldHeight = 8.21; // in meters
  boolean isRed;
  boolean rotateToSpeaker = false;

  Trajectory traj;
  double distancePassed = 0;
  pathPoint[] points;
  double finishVel;

  boolean autoRotate = false;
  double autoRotateVel = 2;

  /**
   * Creates a new path follower using the given points.
   * 
   * @param chassis
   * @param points   from blue alliance
   * @param maxVel   the max velocity in m/s
   * @param maxAccel the max accel in m/s2 (squared)
   * 
   */

  public PathFollow(pathPoint[] points, double velocity) {
    this(RobotContainer.robotContainer.chassis, points, velocity, velocity * 2,
        0, RobotContainer.robotContainer.isRed());
  }

  public PathFollow(pathPoint[] points) {
    this(RobotContainer.robotContainer.chassis, points, ChassisConstants.MAX_DRIVE_VELOCITY,
        ChassisConstants.DRIVE_ACCELERATION,
        0, RobotContainer.robotContainer.isRed());
  }

  public PathFollow(Chassis chassis, pathPoint[] points, double maxVel, double maxAcc, double finishVel) {
    this.points = points;
    this.finishVel = finishVel;

    this.chassis = chassis;

    // gets the wanted angle for the robot to finish the path in

    // creates new coreners array of the "arc points" in the path
    addRequirements(chassis);

    // creates trapezoid object for drive and rotation
    driveTrapezoid = new TrapezoidNoam(maxVel, maxAcc);
    rotationTrapezoid = new TrapezoidNoam(180, 360);

    // calculate the total length of the path
    segments = new Segment[1 + ((points.length - 2) * 2)];

  }

  public PathFollow(Chassis chassis, pathPoint[] points, double maxVel, double maxAcc, double finishVel,
      boolean rotateToSpeaker) {
    this(chassis, points, maxVel, maxAcc, finishVel);
    this.rotateToSpeaker = rotateToSpeaker;
  }

  /*
   * public String currentSegmentInfo() {
   * 
   * if (segments == null)
   * return "";
   * return segments[segmentIndex].toString();
   * }
   */

  @Override
  public void initialize() {
    isRed = RobotContainer.robotContainer.isRed();
    // sets first point to chassis pose to prevent bugs with red and blue alliance
    points[0] = new pathPoint(chassis.getPose().getX(), chassis.getPose().getY(), points[1].getRotation(),
        points[0].getRadius(), false);

    // case for red alliance (blue is the default)
    if (isRed) {
      points[0] = new pathPoint(chassis.getPose().getX(), chassis.getPose().getY(),
          Rotation2d.fromDegrees(180).minus(points[1].getRotation()), points[0].getRadius(), false);
      for (int i = 1; i < points.length; i++) {
        points[i] = new pathPoint(fieldLength - points[i].getX(), points[i].getY(),
            Rotation2d.fromDegrees(180).minus(points[i].getRotation()),
            points[i].getRadius(), points[i].isAprilTag());
      }
    }
    corners = new RoundedPoint[points.length - 2];
    for (int i = 0; i < points.length - 2; i++) {
      corners[i] = new RoundedPoint(points[i], points[i + 1], points[i + 2], points[i].isAprilTag());
    }
    // case for 1 segment, need to create only 1 leg
    if (points.length < 3) {
      segments[0] = new Leg(points[0].getTranslation(), points[1].getTranslation(), points[1].isAprilTag());
      // System.out.println("------LESS THAN 3------");
    }
    // case for more then 1 segment
    else {
      // creates the first leg
      segments[0] = corners[0].getAtoCurveLeg();

      int segmentIndexCreator = 1;
      // creates arc than leg
      for (int i = 0; i < corners.length - 1; i += 1) {

        segments[segmentIndexCreator] = corners[i].getArc();

        segments[segmentIndexCreator + 1] = new Leg(corners[i].getCurveEnd(), corners[i + 1].getCurveStart(),
            points[segmentIndexCreator].isAprilTag());
        segments[segmentIndexCreator].setAprilTagMode(points[segmentIndexCreator].isAprilTag());
        segmentIndexCreator += 2;
      }
      // creates the last arc and leg
      segments[segments.length - 2] = corners[corners.length - 1].getArc();
      segments[segments.length - 1] = corners[corners.length - 1].getCtoCurveLeg();
    }

    // calculates the length of the entire path
    double segmentSum = 0;
    for (Segment s : segments) {
      segmentSum += s.getLength();
    }
    pathLength = segmentSum;
    totalLeft = pathLength;
    segmentIndex = 0;

    List<State> list = new ArrayList<>();
    for (int i = 0; i < points.length; i++) {
      State temp = new State();
      temp.poseMeters = new Pose2d(points[i].getX(), points[i].getY(), new Rotation2d(0));
      list.add(temp);
    }

    // System.out.println("LIST: " + list);

    traj = new Trajectory(list);
    trajField.getObject("TrajTEST").setTrajectory(traj);

    vecVel = new Translation2d(0, 0);
  }

  // calculates the position of the closet april tag and returns it's position
  boolean foundAprilTag = false;

  public Rotation2d getAngleApriltag() {
    Translation2d finalVector = new Translation2d(Integer.MAX_VALUE, Integer.MAX_VALUE);
    // checks the distance from each april tag and finds
    for (int i = 0; i < aprilTagsPositions.length; i++) {

      Translation2d currentAprilTagVector = chassis.getPose().minus(aprilTagsPositions[i]).getTranslation();

      if (currentAprilTagVector.getNorm() < finalVector.getNorm()) {
        finalVector = currentAprilTagVector;
      }

    }
    foundAprilTag = true;

    return finalVector.getAngle();
  }

  public static double convertAlliance(double x) {
    return fieldLength - x;
  }

  public static double fixY(double y) {
    return fieldHeight - y;
  }

  @Override
  public void execute() {

    trajField.setRobotPose(chassis.getPose());

    chassisPose = chassis.getPose();
    // SmartDashboard.putNumber("Angle traj",
    // points[segmentIndex].getRotation().getDegrees());

    // current velocity vector
    Translation2d currentVelocity = new Translation2d(chassis.getChassisSpeeds().vxMetersPerSecond,
        chassis.getChassisSpeeds().vyMetersPerSecond);
    distancePassed = totalLeft - segments[segmentIndex].distancePassed(chassisPose.getTranslation());

    if (segments[segmentIndex].distancePassed(chassisPose.getTranslation()) >= segments[segmentIndex].getLength()
        - distanceOffset) {
      totalLeft -= segments[segmentIndex].getLength();
      if (segmentIndex != segments.length - 1 || segments[segmentIndex].getLength() <= 0.15)
        segmentIndex++;
    }
    driveVelocity = driveTrapezoid.calculate(
        totalLeft - segments[segmentIndex].distancePassed(chassisPose.getTranslation()),
        currentVelocity.getNorm(), finishVel);

    Translation2d velVector = segments[segmentIndex].calc(chassisPose.getTranslation(), driveVelocity);

    // System.out.println("APRILTAG MODE: " +
    // segments[segmentIndex].isAprilTagMode());
    if (segments[segmentIndex].isAprilTagMode()) {
      if (!foundAprilTag)
        wantedAngle = getAngleApriltag();
    }

    else {
      wantedAngle = points[segmentIndex].getRotation();
    }

    if (totalLeft <= 0.1)
      velVector = new Translation2d(0, 0);
    ChassisSpeeds speed = new ChassisSpeeds(velVector.getX(), velVector.getY(), 0);
    if (rotateToSpeaker) {
      chassis.setVelocitiesRotateToSpeaker(speed);
    } else if(autoRotate) {
      speed.omegaRadiansPerSecond = autoRotateVel;
      chassis.setVelocities(speed);
    } else {
      chassis.setVelocitiesRotateToAngle(speed, wantedAngle);
    }

  }

  public PathFollow setAutoRotate(double rate) {
    autoRotate = true;
    autoRotateVel = rate;
    return this;
  }

  @Override
  public void end(boolean interrupted) {
    if(finishVel == 0) chassis.stop();
    driveTrapezoid.debug = false;
    // .useAcceleration = true;
  }

  @Override
  public boolean isFinished() {
    return totalLeft <= 0.1;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // builder.addStringProperty("Current Segment", () -> currentSegmentInfo(),
    // null);
    super.initSendable(builder);
    builder.addDoubleProperty("Distance Passed", () -> {
      return distancePassed;
    }, null);
    builder.addDoubleProperty("Total Left", () -> {
      return totalLeft;
    }, null);
    builder.addDoubleProperty("Velocity", () -> {
      return driveVelocity;
    }, null);
    builder.addDoubleProperty("Rotation Velocity", () -> {
      return Math.toDegrees(rotationVelocity);
    }, null);
    builder.addDoubleProperty("Angle", () -> {
      return chassisPose.getRotation().getDegrees();
    }, null);
    builder.addDoubleProperty("Pose X", () -> chassis.getPose().getX(), null);
    builder.addDoubleProperty("Pose Y", () -> chassis.getPose().getY(), null);
  }

  public void printSegments() {
    for (Segment s : segments) {
     System.out.println(s);
    }
  }
}
