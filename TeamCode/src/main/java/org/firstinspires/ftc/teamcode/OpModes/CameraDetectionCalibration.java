package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.internals.AprilTagDetector;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.utils.AprilTagMath;
import org.openftc.apriltag.AprilTagDetection;

@TeleOp(name = "cameraDetection", group = "autoTuning")
@Config
public class CameraDetectionCalibration extends LinearOpMode {
    public static int APRIL_TAG_ID = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, null);
        ControlHub.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AprilTagDetector.init(hardwareMap);
        FtcDashboard.getInstance().startCameraStream(AprilTagDetector.camera, 30);

        waitForStart();
        ElapsedTime f = new ElapsedTime();

        while (opModeIsActive()){
            AprilTagDetection d = null;
            for(AprilTagDetection detection : AprilTagDetector.getDetections())
                if(detection.id == APRIL_TAG_ID) d = detection;
            Pose2d pose = new Pose2d();
            if(d != null) {
                Pose2d poseFromAprilTag = AprilTagMath.poseFromTag(pose, d);
                ControlHub.telemetry.addData("x", poseFromAprilTag.getX());
                ControlHub.telemetry.addData("y", poseFromAprilTag.getY());
                ControlHub.telemetry.addData("heading", poseFromAprilTag.getHeading());
            }
            ControlHub.telemetry.addData("freq", 1/f.seconds());
            f.reset();
            ControlHub.telemetry.update();
        }

    }
}
