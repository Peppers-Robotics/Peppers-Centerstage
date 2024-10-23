package org.firstinspires.ftc.teamcode.Auto;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.AprilTagDetector;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.AprilTagMath;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.File;
import java.io.IOException;

@Autonomous(name = "BlueFar", group = "auto")
@Config
@Disabled
public class BlueFar extends LinearOpMode {
    public static Pose2d
            MiddlePurple = new Pose2d(48, -10.5, -Math.toRadians(227)),
            MiddleYellow = new Pose2d(37.5, 85.3, -Math.toRadians(243)),


    Stack = new Pose2d(51, -18.5, -Math.toRadians(270)),
            Backdrop = new Pose2d(45.5, 86, -Math.toRadians(243)),
            Stack2 = new Pose2d(51, -18.5, -Math.toRadians(270)),
            StackGate = new Pose2d(52, 4, Math.PI/2),
            BackdropGate = new Pose2d(52, 65, Math.PI/2);

    SampleMecanumDriveCancelable drive;
    OutTakeMTI outtake;
    Intake intake;
    int order = 0, intakeActive = 0, pixelsInStack = 5, desiredID = 5, cycle = 0;
    boolean isInPreload = true, cameraRelocalize = false, pixelUpdate = false, ack = false, relocalized = false;

    @Override
    public void runOpMode() throws InterruptedException {
        File file = new File(Environment.getExternalStorageDirectory(), OutTakeMTI.cacheFileName);

        if(file.exists()){
            file.delete();
        }
        try {
            file.createNewFile();
        } catch (IOException e) {
            RobotLog.e("file not found");
        }

        drive = new SampleMecanumDriveCancelable(hardwareMap);
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap, drive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);
        ControlHub.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ExpansionHub.setInitialBackdropAngleRelativeToBot(90);

//        AprilTagDetector.init(hardwareMap);
//        FtcDashboard.getInstance().startCameraStream(AprilTagDetector.camera, 120);

        outtake = new OutTakeMTI();
        intake = new Intake();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    isInPreload = true;
                    outtake.setToPurplePlacing();
                    OutTakeMTI.arm.rotationIndex = 0;
                    desiredID = 2;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 3, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(MiddlePurple)
                .resetConstraints()

                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    order = 1;
                })
                .lineToLinearHeading(Stack)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    Controls.RetractElevatorAck = false;
                    Controls.RetractElevator = true;
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence takeFromStack = drive.trajectorySequenceBuilder(Stack2)
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                    intake.setPixelStackPosition(pixelsInStack);
                    pixelUpdate = false;
                })
                .forward(2)
                .back(2)
                .forward(2)
                .back(2)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    if(!pixelUpdate)
                        pixelsInStack --;
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(Backdrop)
                .setReversed(true)

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(new Pose2d(BackdropGate.getX(), BackdropGate.getY(), BackdropGate.getHeading()), -Math.toRadians(90))

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(StackGate, -Math.toRadians(90))

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(45))
                .splineToSplineHeading(Stack2, -Math.toRadians(90))
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                    intake.setPixelStackPosition(pixelsInStack);
                    pixelUpdate = false;
                    ack = false;
                })

                .build();

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(Stack2)
                .setReversed(false)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    cycle++;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intakeActive = 0;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(new Pose2d(BackdropGate.getX(), BackdropGate.getY(), BackdropGate.getHeading()), Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    OutTakeMTI.State.level = 5.8;
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
                })

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(Backdrop, Math.toRadians(110))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    Controls.DropRightAck = false;
                    Controls.DropRight = true;
                })
                .build();

        TrajectorySequence middleYellowGoTo = drive.trajectorySequenceBuilder(Stack)
                .setReversed(false)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 4, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToLinearHeading(StackGate, Math.toRadians(90))
                .addTemporalMarker(() -> {
                    intakeActive = 0;
                    OutTakeMTI.arm.rotationIndex = 0;
                })

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(60))
                .splineToSplineHeading(new Pose2d(BackdropGate.getX(), BackdropGate.getY(), BackdropGate.getHeading()), Math.toRadians(90))

                .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> {
                    OutTakeMTI.State.level = 4;
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
//                    cameraRelocalize = true;
                })
//                .build();
//        TrajectorySequence middleYellowPlace = drive.trajectorySequenceBuilder(middleYellowGoTo.end())

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 3.1415, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(MiddleYellow, Math.toRadians(160))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    isInPreload = false;
                })
                .lineToLinearHeading(Backdrop)
                .addTemporalMarker(() -> {
                    Controls.DropRightAck = false;
                    Controls.DropRight = true;
                })
                .waitSeconds(0.1)
                .build();

        while (opModeInInit()){
            outtake.update();
            telemetry.update();
        }

        ElapsedTime AutoTime = new ElapsedTime();
        drive.followTrajectorySequenceAsync(middle);
        long time1 = System.currentTimeMillis();

        while (opModeIsActive()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();
            ExpansionHub.ImuYawAngle = Math.toDegrees(drive.getPoseEstimate().getHeading()) - ExpansionHub.beforeReset;
            if((System.currentTimeMillis() - time1) >= 1.0 / 4){
                double Yawn = ExpansionHub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Yawn));
//                ExpansionHub.ImuYawAngle = Yawn - ExpansionHub.beforeReset;
                time1 = System.currentTimeMillis();
            }
            Controls.Intake = false;
            Controls.RevIntake = false;
            if(intakeActive == 1){
                Controls.Intake = true;
            } else if(intakeActive == -1){
                Controls.RevIntake = true;
            }

            if (order == 1 && OutTakeMTI.isFullOfPixels()) {
                pixelsInStack --;
                drive.breakFollowing();
                order ++;
                pixelUpdate = true;
            }
            if(order == 1 && OutTakeMTI.hasAPixel() && !isInPreload && !ack){
                pixelsInStack --;
                drive.breakFollowing();
                ack = true;
            }

            if(cameraRelocalize) {
                Pose2d currentPosition = drive.getPoseEstimate();
                boolean foundAprilTag = false;
                for (AprilTagDetection a : AprilTagDetector.getDetections()) {
                    if (a.id == desiredID) {
                        foundAprilTag = true;
                        currentPosition = AprilTagMath.poseFromTag(currentPosition, a);
                        relocalized = true;
                        break;
                    }
                }
                if (foundAprilTag) {
                    drive.setPoseEstimate(currentPosition);
                    cameraRelocalize = false;
                }
            } else if(relocalized && isInPreload) {
//                drive.followTrajectorySequenceAsync(middleYellowPlace);
            }

            if(!drive.isBusy() && !cameraRelocalize){
                switch (order) {
                    case 0:
                        if(AutoTime.seconds() <= 25) {
                            drive.followTrajectorySequenceAsync(goToStack);
                            order++;
                        }
                        break;
                    case 1:
                        drive.followTrajectorySequenceAsync(takeFromStack);
                        break;
                    case 2:
                        if (isInPreload) {
                            drive.followTrajectorySequenceAsync(middleYellowGoTo);
                        } else drive.followTrajectorySequenceAsync(goToBackdrop);
                        order = 0;
                        break;
                }
            }
            drive.update();
            outtake.update();
            intake.update();
            cn.loop();
            ControlHub.telemetry.addData("order", order);
            ControlHub.telemetry.addData("pixelsInStack", pixelsInStack);
            ControlHub.telemetry.addData("cycles", cycle);

            ControlHub.telemetry.update();
        }

    }
}
