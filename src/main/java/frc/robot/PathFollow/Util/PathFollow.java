package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.chassis.ChassisConstants.*;
import edu.wpi.first.math.trajectory.Trajectory.State;

import java.util.ArrayList;
import java.util.List;

import frc.robot.RobotContainer;
import frc.robot.chassis.subsystems.Chassis;

import static frc.robot.PathFollow.Util.PathsConstants.*;

public class PathFollow extends Command {

  Chassis chassis;
  RoundedPoint[] corners;
  Pose2d closestAprilTag = new Pose2d();
  Pose2d chassisPose = new Pose2d();
  double pathLength;
  double distanceLeft;
  int segmentIndex = 0;

  double maxVel;

  Segment[] segments;
  Translation2d vecVel;
  Rotation2d wantedAngle;

  PathsTrapezoid driveTrapezoid;
  PathsTrapezoid rotationTrapezoid;
  Field2d trajField = new Field2d();

  double driveVelocity = 0;
  double rotationVelocity = 0;

  boolean isRed;
  Trajectory traj;

  double distancePassed = 0;
  pathPoint[] points;
  double finishVel;
  

  public PathFollow(pathPoint[] points, double velocity) {
    this(RobotContainer.robotContainer.chassis, points, velocity, velocity * 2, 0);
  }

  public PathFollow(pathPoint[] points) {
    this(RobotContainer.robotContainer.chassis, points, MAX_VELOCITY, ACCEL,0);
  }

  public PathFollow(Chassis chassis, pathPoint[] points, double maxVel, double maxAcc, double finishVel) {
    this.points = points;
    this.finishVel = finishVel;
    this.chassis = chassis;
    this.maxVel = maxVel;
    addRequirements(chassis);

    // creates trapezoid object for drive and rotation
    driveTrapezoid = new PathsTrapezoid(maxVel, maxAcc);
    rotationTrapezoid = new PathsTrapezoid(MAX_ROTATION_VELOCITY, ROTATION_ACCEL);

    // calculate the total length of the path
    segments = new Segment[1 + ((points.length - 2) * 2)];

  }

  private double convertAlliance(double x) {
    return FIELD_LENGTH - x;
  }



  /*
   * public String currentSegmentInfo() {
   * 
   * if (segments == null)
   * return "";
   * return segments[segmentIndex].toString();
   * }
   */


   private void setMaxRadiusPoints(){
    for(int i = 0; i < points.length; i++){
      points[i].setRadius(getMaxRadius(maxVel));
    }
   }

  private void setFirstPoint(boolean isRed){

    points[0] = isRed ? new pathPoint(convertAlliance(chassis.getPose().getX()), chassis.getPose().getY(), points[1].getRotation(), points[0].getRadius())
      : new pathPoint(chassis.getPose().getX(), chassis.getPose().getY(), points[1].getRotation(), points[0].getRadius());

  }
  
  private void convertPoints(){
    for(int i = 1; i < points.length; i++){
      points[i] = new pathPoint(new Translation2d(convertAlliance(points[i].getX()), points[i].getY()), points[i].getRotation(), points[i].getRadius());
    }
  }

  private void createCorners(){
    for (int i = 0; i < points.length - 2; i++) {
      corners[i] = new RoundedPoint(points[i], points[i + 1], points[i + 2], points[i].isAprilTag());
    }
  }

  private void createSegments(){
    //case for only leg
    if (points.length < 3) {
      segments[0] = new Leg(points[0].getTranslation(), points[1].getTranslation());
    
    }
    // case for more then 1 segment
    else {
      // creates the first leg
      segments[0] = corners[0].getAtoCurveLeg();

      int segmentIndexCreator = 1;
      // creates arc than leg
      for (int i = 0; i < corners.length - 1; i++) {

        segments[segmentIndexCreator] = corners[i].getArc();
        segments[segmentIndexCreator + 1] = new Leg(corners[i].getCurveEnd(), corners[i + 1].getCurveStart());
        segmentIndexCreator += 2;
      }
      // creates the last arc and leg
      segments[segments.length - 2] = corners[corners.length - 1].getArc();
      segments[segments.length - 1] = corners[corners.length - 1].getCtoCurveLeg();
    }
  }

  private double calcPathLength(){
    double sum = 0;
    for (Segment s : segments) {
      sum += s.getLength();
    }
    return sum;
  }
  
  private void setTrajectoryField(){
    List<State> list = new ArrayList<>();
    for (int i = 0; i < points.length; i++) {
      State temp = new State();
      temp.poseMeters = new Pose2d(points[i].getX(), points[i].getY(), new Rotation2d(0));
      list.add(temp);
    }
    traj = new Trajectory(list);
    trajField.getObject("TrajTEST").setTrajectory(traj);
    
  }

  public double getMaxRadius(double velocity){
    return (velocity * velocity) / MAX_RADIAL_ACCEL;
  }
  
  @Override
  public void initialize() {
    isRed = RobotContainer.robotContainer.isRed();
    // sets first point to chassis pose to prevent bugs with red and blue alliance
    setFirstPoint(isRed);

    // case for red alliance (blue is the default)
    if (isRed) {
      convertPoints();
    }

    corners = new RoundedPoint[points.length - 2];
    setMaxRadiusPoints();
    createCorners();
    createSegments();

    // calculates the length of the entire path
    pathLength = calcPathLength();
    distanceLeft = pathLength;
    segmentIndex = 0;

  
    setTrajectoryField();

    vecVel = new Translation2d(0, 0);
  }

  // calculates the position of the closet april tag and returns it's position

  boolean foundAprilTag = false;

  private boolean finishedSegment(){
    return segments[segmentIndex].distancePassed(chassisPose.getTranslation()) >= segments[segmentIndex].getLength() - DISTANCE_OFFSET;
  }
  private boolean isLastSegment(){
    return segmentIndex == segments.length - 1;
  }
  private boolean isSegmentTooShort(){
    return segments[segmentIndex].getLength() <= MIN_SEGMENT_LENGTH;
  }

  @Override
  public void execute() {

    trajField.setRobotPose(chassis.getPose());
    chassisPose = chassis.getPose();

    Translation2d currentVelocity = chassis.getVelocity();
    distanceLeft -= segments[segmentIndex].distancePassed(chassisPose.getTranslation());

    if (finishedSegment() && (!isLastSegment() || isSegmentTooShort())) {
      segmentIndex++;
    }

    driveVelocity = driveTrapezoid.calc(distanceLeft, currentVelocity.getNorm(), finishVel);
    rotationVelocity = rotationTrapezoid.calc(wantedAngle.getRadians(), chassis.getChassisSpeeds().omegaRadiansPerSecond, 0);
    Translation2d velVector = segments[segmentIndex].calc(chassisPose.getTranslation(), driveVelocity);

    wantedAngle = points[segmentIndex].getRotation();
    
    ChassisSpeeds speed = new ChassisSpeeds(velVector.getX(), velVector.getY(), rotationVelocity);
    chassis.setVelocities(speed);

  }


  @Override
  public void end(boolean interrupted) {
    if(finishVel == 0) chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return distanceLeft <= FINISH_OFFSET;
  }

  public void printSegments() {
    for (Segment s : segments) {
     System.out.println(s);
    }
  }
}
