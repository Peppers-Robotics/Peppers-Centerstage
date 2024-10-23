package org.firstinspires.ftc.teamcode.Auto.oldAuto;

import androidx.core.view.accessibility.AccessibilityViewCommand;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.AprilTagDetector;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.utils.UltraSonicSensor;

import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "Relocalization Test", group = "test")
@Config
public class RelocalizationTest extends LinearOpMode {
    SampleMecanumDriveCancelable drive;
    public static int DesiredId = 5;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ExpansionHub e = new ExpansionHub(hardwareMap, drive.getLocalizer());
        ControlHub c = new ControlHub(hardwareMap);

        double readValue = 0;

        Rev2mDistanceSensor sensor = hardwareMap.get(Rev2mDistanceSensor.class, "redSensor");



        waitForStart();


        long time = System.currentTimeMillis();
        long time1 = System.currentTimeMillis();
        long time2 = System.currentTimeMillis();
        double seconds;
        double secondsToMeshIMU = 0;
        double ProportionalTerm = 0.4;

        while (!isStopRequested()){
//            e.update(false, drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());
            secondsToMeshIMU = (System.currentTimeMillis() - time2) / 1000.f;
            seconds = (System.currentTimeMillis() - time1) / 1000.f;
            if(secondsToMeshIMU >= 2.f) {
                Pose2d robotPose1 = drive.getPoseEstimate();
//                drive.setPoseEstimate(new Pose2d(robotPose1.getX(), robotPose1.getY(), ExpansionHub.yawnAngle));
                time2 = System.currentTimeMillis();
            }
            if(seconds >= 1/5.f){
                Pose2d robotPose = drive.getPoseEstimate();
                double distance = sensor.getDistance(DistanceUnit.INCH);
                distance *= Math.cos(robotPose.getHeading());

                drive.setPoseEstimate(new Pose2d(distance, robotPose.getY(), robotPose.getHeading()));
                time1 = System.currentTimeMillis();
            }
            telemetry.addData("freq", 1000.f / ((System.currentTimeMillis() - time)));
            time = System.currentTimeMillis();

            drive.update();
            telemetry.update();
        }
        AprilTagDetector.camera.closeCameraDevice();
    }
}
