package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.AprilTagDetector;
import org.firstinspires.ftc.teamcode.utils.AprilTagMath;
import org.openftc.apriltag.AprilTagDetection;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationApril extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        //AprilTagDetector.init(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime time = new ElapsedTime();

        waitForStart();

        AprilTagDetector.camera.pauseViewport();
        AprilTagDetector.camera.stopStreaming();
        AprilTagDetector.camera.stopRecordingPipeline();

            while (!isStopRequested()) {

//                AprilTagDetection[] detections = AprilTagDetector.getDetections();
//                AprilTagDetection detection = null;
//
//                for (AprilTagDetection det : detections)
//                    if (det.id == 6) detection = det;

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
//                if (detection != null) {
//                    Pose2d pose = AprilTagMath.poseFromTag(poseEstimate, detection, 6);
//                    telemetry.addData("x", pose.getX());
//                    telemetry.addData("y", pose.getY());
//                    telemetry.addData("heading", pose.getHeading());
//                }
                telemetry.addData("loop time ms:", time.milliseconds());
                time.reset();
                telemetry.update();
            }


    }
}
