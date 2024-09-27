package frc.robot.utils.ImageProsesing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class CalcExpPose{
    private Pose2d[] pointsArr;
    private double[] timeArr;
    private double velocity;
    private double angle;
    private double sumXLocation = 0;
    private double sumYLocation = 0;

    public CalcExpPose(Pose2d[] pointsArr, double[] timeArr, double velocity, double angle)
    {
        this.pointsArr = pointsArr;
        this.timeArr = timeArr;
        this.velocity = velocity;
        this.angle = angle;
    }
    public Pose2d GetExpectedPos()
    {

        for (int i = 0; i < pointsArr.length; i++)
        {
            double xLocation = pointsArr[i].getTranslation().getX() + Math.cos(angle) * velocity * timeArr[i];
            double yLocation = pointsArr[i].getTranslation().getY() + Math.sin(angle) * velocity * timeArr[i];
            sumXLocation += xLocation;
            sumYLocation += yLocation;
        }
        double avgX = sumXLocation / pointsArr.length;
        double avgY = sumYLocation / pointsArr.length;
        return new Pose2d(new Translation2d(avgX, avgY), new Rotation2d());
    }
}