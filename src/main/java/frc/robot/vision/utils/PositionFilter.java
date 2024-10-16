// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.filter.MedianFilter;
import java.util.LinkedList;
import java.util.Queue;

public class PositionFilter {
    private MedianFilter xFilter;
    private MedianFilter yFilter;
    private MedianFilter rotationFilter;
    private Queue<Pose2d> poseBuffer;
    private int bufferSize;
    private double maxVelocity;
    private double maxRotationVelocity;

    public PositionFilter(int windowSize, int bufferSize, double maxVelocity, double maxRotationVelocity) {
        this.xFilter = new MedianFilter(windowSize);
        this.yFilter = new MedianFilter(windowSize);
        this.rotationFilter = new MedianFilter(windowSize);
        this.poseBuffer = new LinkedList<>();
        this.bufferSize = bufferSize;
        this.maxVelocity = maxVelocity;
        this.maxRotationVelocity = maxRotationVelocity;
    }

    public Pose2d update(Pose2d newPose, double deltaTime) {
        if (poseBuffer.size() >= bufferSize) {
            Pose2d oldestPose = poseBuffer.poll();
            double dx = (newPose.getX() - oldestPose.getX()) / deltaTime;
            double dy = (newPose.getY() - oldestPose.getY()) / deltaTime;
            double dTheta = (newPose.getRotation().getRadians() - oldestPose.getRotation().getRadians()) / deltaTime;

            if (Math.hypot(dx, dy) > maxVelocity || Math.abs(dTheta) > maxRotationVelocity) {
                return poseBuffer.peek(); // Return last valid pose if velocity exceeds limits
            }
        }

        poseBuffer.offer(newPose);

        double filteredX = xFilter.calculate(newPose.getX());
        double filteredY = yFilter.calculate(newPose.getY());
        double filteredRotation = rotationFilter.calculate(newPose.getRotation().getRadians());

        return new Pose2d(filteredX, filteredY, new Rotation2d(filteredRotation));
    }
}