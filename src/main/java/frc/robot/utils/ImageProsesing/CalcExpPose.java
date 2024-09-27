package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class CalcExpectedPose{
    private Pose2d[] pointsArr;
    private double[] timeArr;
    private double velocity;
    private double angle;
    private double sumXLocation = 0;
    private double sumYLocation = 0;

    public CalcExpectedPose(Pose2d[] pointsArr, double[] timeArr, double velocity, double angle)
    {
        this.pointsArr = pointsArr;
        this.timeArr = timeArr;
        this.velocity = velocity;
        this.angle = angle;
    }
    public Pose2d GetExpectedPos()
    {

        for (int i = 0; i < pointsArr.Length; i++)
        {
            double xLocation = pointsArr[i].getTranslation().getX() + Math.cos(angle) * velocity * timeArr[i];
            double yLocation = pointsArr[i].getTranslation().getY() + Math.sin(angle) * velocity * timeArr[i];
            sumXLocation += xLocation;
            sumYLocation += yLocation;
        }
        double avgX = sumXLocation / pointsArr.Length;
        double avgY = sumYLocation / pointsArr.Length;
        return new Pose2d(avgX, avgY);
    }
}