package org.firstinspires.ftc.teamcode.Auto.oldAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "OutTakeAutoTest", group = "test")
public class OutTakeAutoTest extends LinearOpMode {
    public SampleMecanumDriveCancelable drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, drive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);

        ControlHub.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OutTakeMTI outTake = new OutTakeMTI();

        TrajectorySequence t = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    outTake.setToPurplePlacing();
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    OutTakeMTI.State.level = 1;
                    OutTakeMTI.elevator.setTargetPosition(2.5 * OutTakeMTI.STEP);
                    OutTakeMTI.align = true;
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                })
                .lineToLinearHeading(new Pose2d(2, 0, 0))
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    OutTakeMTI.driverUpdated = true;
//                    Controls.ExtendElevatorAck = false;
//                    Controls.ExtendElevator = true;
                })
                .waitSeconds(0.2)
                .build();

        waitForStart();
        drive.followTrajectorySequenceAsync(t);

        while (opModeIsActive()){

            outTake.update();
            drive.update();
            cn.loop();
        }
    }
}
