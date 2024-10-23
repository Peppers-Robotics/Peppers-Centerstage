package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;

@Config
public class AprilTagMath {
    public static double distanceToCenter = 18.6 / 2.54;
    public static double AprilTagToINCHES = 25.4; // assume distance is in meters
    public static double[] TAG_X_OFFSET = {0, 20.9, 26.9, 33.4, 34.9, 28.9, 24.9};
    public static double[] TAG_Y_OFFSET = {0, 0, 0, 0, -43, -43, -43};

    public static Pose2d poseFromTag(Pose2d robotPose, AprilTagDetection detection) {

        // these are the x and y positions of the AprilTag relative
        // to the robot's global shutter camera
        double tagX = detection.pose.z * AprilTagToINCHES;
        double tagY = detection.pose.x * AprilTagToINCHES;

        // this is the current robot heading
        double robotHeading = robotPose.getHeading();

        // because the global shutter camera is rotated at the same angle
        // as the robot, we need to recompute the position vector of
        // the AprilTag relative to our global starting position
        double x_displacement = cos(robotHeading) * tagX - sin(robotHeading) * tagY;
        double y_displacement = cos(robotHeading) * tagY + sin(robotHeading) * tagX;

        // because the global shutter camera is not located at the
        // center the robot, we need add an offset vector to translate the
        // vector to the center of the robot
        double x_to_center = cos(robotHeading) * distanceToCenter;
        double y_to_center = sin(robotHeading) * distanceToCenter;

        // return the final positions with the global offset of the AprilTags on the field
        return new Pose2d(-(x_displacement + x_to_center) + TAG_X_OFFSET[detection.id], -(y_displacement + y_to_center) + TAG_Y_OFFSET[detection.id], robotHeading);
    }
}
