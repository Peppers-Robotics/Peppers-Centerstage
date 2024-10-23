package org.firstinspires.ftc.teamcode.Auto.oldAuto;

import android.graphics.LinearGradient;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.analysis.function.Exp;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.detectionPipelines.RedCloseDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "redclose shit")
@Config
@Disabled
public class RedCloseN extends LinearOpMode {
    private RedCloseDetectionPipeline detection;

    public static double purpleMiddle_x = 22, purpleMiddle_y = 0;
    public static double yellowMiddle_x = 24, yellowMiddle_y = -31;

    public static double preUnderTruss_x = 5.4, preUnderTruss_y = 4;
    public static double postUnderTruss_x = 6, postUnderTruss_y = 46.2;
    public static double stack_x = 25, stack_y = 77, stack_h = Math.toRadians(240);
    //Poses:
    private Pose2d finishedPreloadPosition;

    private static OutTake outtake;
    private static Intake intake;
    private static SampleMecanumDriveCancelable drive;
    private boolean intake_active = false, toStack = false, switchToStackTaking, outTake_intake = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, drive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);
        ControlHub.telemetry = telemetry;
        outtake = new OutTake(hardwareMap);
        intake = new Intake();
        OutTake.outTakeExtension.MOTION_PROFILED = true;
        ExpansionHub.extension_length = 6900;
        OutTake.State.level = 1;

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    Controls.SetOuttakeToPurplePlacing = true;
                })
                .lineToLinearHeading(new Pose2d(purpleMiddle_x, purpleMiddle_y, 0))
                .waitSeconds(0.03)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.07)
                .addTemporalMarker(() -> {
                    outtake.setToTablePlacement();
                })
                .lineToLinearHeading(new Pose2d(yellowMiddle_x, yellowMiddle_y, 5.1282))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    toStack = true;
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .splineTo(new Vector2d(preUnderTruss_x, preUnderTruss_y), Math.PI/2)
                .setConstraints(SampleMecanumDriveCancelable.getVelocityConstraint(80, 5, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDriveCancelable.getAccelerationConstraint(75))
                .lineToConstantHeading(new Vector2d(postUnderTruss_x, postUnderTruss_y))
                .addTemporalMarker(() -> {
                    intake_active = true;
                    intake.setPixelStackPosition(0);
                })
                .resetConstraints()
                .splineToSplineHeading(new Pose2d(stack_x, stack_y, stack_h), stack_h + Math.PI)
                .addTemporalMarker(() -> {
                    switchToStackTaking = true;
                })
                .waitSeconds(0.01)
                .build();

        TrajectorySequence placePixelsSeqence = drive.trajectorySequenceBuilder(finishedPreloadPosition)
                .addTemporalMarker(() -> {
                    switchToStackTaking = false;
                    intake_active = false;
                    outTake_intake = true;
                })
                .splineToLinearHeading(new Pose2d(postUnderTruss_x, postUnderTruss_y, Math.PI/2), Math.PI/2)
                .build();

        waitForStart();
        drive.followTrajectorySequenceAsync(middle);
        finishedPreloadPosition = middle.end();

        while(opModeIsActive()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();
            ExpansionHub.ImuYawAngle = Math.toDegrees(drive.getLocalizer().getPoseEstimate().getHeading()) + 90;
            Controls.Intake = intake_active;
            Controls.RevIntake = outTake_intake;

            if(switchToStackTaking){
                if(OutTake.onePixel() || OutTake.fullPixel()) intake.setPixelStackPosition(intake.getPixelStackPosition() + 1);
                if(OutTake.fullPixel()){
                    drive.followTrajectorySequenceAsync(placePixelsSeqence);
                }


            }


            drive.update();

            outtake.update();
            outtake.update_values();

            intake.update();
            intake.update_values();

            outtake.runTelemetry();

            telemetry.update();
            telemetry.addData("imu angle", ExpansionHub.ImuYawAngle);
            cn.loop();
        }

    }
}
