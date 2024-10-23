package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.drawable.GradientDrawable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.AprilTagDetector;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp(name = "aprilTagDetection")
@Config
public class AprilTag extends LinearOpMode {
    final double METERS_TO_INCHES = 39.37;
    public static double TARGET_ID = 2;
    public static double OFFSET_X = 23; // 81.2
    public static double OFFSET_Y = 39; // 22.2
    private Localizer localizer;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        AprilTagDetector.init(hardwareMap);
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            localizer.update();
            Pose2d pose = localizer.getPoseEstimate();

            telemetry.addData("robot_x: ", pose.getX());
            telemetry.addData("robot_y: ", pose.getY());
            telemetry.addData("robot_heading: ", pose.getHeading());

            AprilTagDetection[] detections = AprilTagDetector.getDetections();
            AprilTagDetection target_tag = null;
            for (AprilTagDetection detected : detections) {
                if(detected.id == TARGET_ID) {
                    target_tag = detected;

                    telemetry.addData("id: ", detected.id);
                    telemetry.addData("x: ", detected.center.x * METERS_TO_INCHES); // this is X in global pose
                    telemetry.addData("y: ", detected.center.y * METERS_TO_INCHES);
                    telemetry.addData("z: ", detected.pose.z * METERS_TO_INCHES); // this is Y in global pose

                }

            }

            if(target_tag != null) {
                telemetry.addData("pose_from_tag_x: ", OFFSET_X - target_tag.pose.x * METERS_TO_INCHES);
                telemetry.addData("pose_from_tag_y: ", OFFSET_Y - target_tag.pose.z * METERS_TO_INCHES);
            }

            telemetry.update();
        }
    }
}
