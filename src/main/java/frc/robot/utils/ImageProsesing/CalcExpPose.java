package frc.robot.utils.ImageProsesing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class CalcExpPose{
    private Pose2d[] pointsArr;
    private double[] timeArr;
    private double velocity;
    private double angle;
    private double sumXLocation;
    private double sumYLocation;

    public CalcExpPose(Pose2d[] pointsArr, double[] timeArr, double velocity)
    {
        this.pointsArr = pointsArr;
        this.timeArr = timeArr;
        this.velocity = velocity;
        this.angle = pointsArr[0].getRotation().getDegrees();
    }
    public Pose2d GetExpectedPos()
    {
        sumXLocation = 0;
        sumYLocation = 0;

        //Postion in set amount of seconds
        double setTime = timeArr[0] + 2;

        for (int i = 0; i < pointsArr.length; i++)
        {
            double time = setTime - timeArr[i];
            double xLocation = pointsArr[i].getTranslation().getX() + Math.cos(angle) * velocity * time;
            double yLocation = pointsArr[i].getTranslation().getY() + Math.sin(angle) * velocity * time;
            sumXLocation += xLocation;
            sumYLocation += yLocation;
        }
        double avgX = sumXLocation / pointsArr.length;
        double avgY = sumYLocation / pointsArr.length;
        return new Pose2d(new Translation2d(avgX, avgY), new Rotation2d());
    }
}
