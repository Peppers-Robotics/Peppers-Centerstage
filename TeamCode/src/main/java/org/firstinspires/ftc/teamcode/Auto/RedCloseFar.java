package org.firstinspires.ftc.teamcode.Auto;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.detectionPipelines.RedCloseDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;
import java.io.IOException;

@Autonomous(name = "RedCloseFar", group = "auto", preselectTeleOp = ".pipers \uD83C\uDF36")
@Disabled
public class RedCloseFar extends LinearOpMode {

    public static Pose2d
            MiddlePurple = new Pose2d(22, -1, 0),
            MiddleYellow = new Pose2d(38.5, -31.5, Math.toRadians(234)),

    RightPurple = new Pose2d(44.7, -3, Math.toRadians(190)),
            RightYellow = new Pose2d(38, -30.5, Math.toRadians(233)),

    LeftPurple = new Pose2d(12, 6, Math.toRadians(45)),
            LeftYellow = new Pose2d(39.5, -30.5, Math.toRadians(245)),

    BackDropGate = new Pose2d(51, 5, Math.toRadians(270)),
            StackGate = new Pose2d(51, 43, Math.toRadians(270)),
            Stack = new Pose2d(50.5, 76.5, Math.toRadians(270)),
            Stack2 = new Pose2d(41, 77, Math.toRadians(291)),
            Backdrop = new Pose2d(46, -31, Math.toRadians(240));

    SampleMecanumDriveCancelable drive;
    OutTakeMTI outTake;
    Intake intake;
    int intakeActive = 0, pixelsInStack = 5, order = 0;
    private boolean ack = false, pixelsUpdated = false, isInPreloadPhase = false, firstPathAfterPreload = false;
    private boolean transtStack = false, secondStackCycle = false;
    int cycle = 0;

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
        ExpansionHub e = new ExpansionHub(hardwareMap, drive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);
        ControlHub.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ExpansionHub.setInitialBackdropAngleRelativeToBot(-90);

        outTake = new OutTakeMTI();
        intake = new Intake();

        OutTakeMTI.timeToDrop = 0.3;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        RedCloseDetectionPipeline detector = new RedCloseDetectionPipeline(ControlHub.telemetry, false);

        camera.setPipeline(detector);
        // ------------------ OpenCv code
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(432, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                // ------------------ Tzeapa frate
            }

        });
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    isInPreloadPhase = true;
                    OutTakeMTI.State.level = 2;
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
                    OutTakeMTI.driverUpdated = false;
                })
                .lineToLinearHeading(RightYellow)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                })
                .waitSeconds(0.1)
                .addTemporalMarker( () -> {
                    OutTakeMTI.align = false;
                    OutTakeMTI.elevator.setTargetPosition(0);
                    OutTakeMTI.arm.setOrientation(0);
                    OutTakeMTI.arm.rotationIndex = 4;
                    outTake.setToPurplePlacing();
                    OutTakeMTI.driverUpdated = false;
                    OutTakeMTI.arm.rotationIndex = 4;
                })
                .lineToLinearHeading(RightPurple)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                })
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                    Controls.RetractElevatorAck = false;
                    order = 1;
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(Backdrop)
                .build();
//                .setReversed(true)
//                .addTemporalMarker(() -> {
//                    firstPathAfterPreload = false;
//                })
//
//                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
//                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
//                .lineToSplineHeading(new Pose2d(StackGate.getX(), StackGate.getY(), StackGate.getHeading()))
//                .addTemporalMarker(() -> {
//                    intake.setPixelStackPosition(pixelsInStack);
//                    intakeActive = 1;
//                    pixelsUpdated = false;
//                    isInPreloadPhase = false;
//                    ack = false;
//                })
//                .splineToSplineHeading(Stack, Math.toRadians(90))
//
//
//                .build();


        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    outTake.setToPurplePlacing();
                    isInPreloadPhase = true;
                })
                .lineToLinearHeading(LeftPurple)
                .UNSTABLE_addDisplacementMarkerOffset(0.2, () -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    OutTakeMTI.State.level = 1;
                    outTake.updateElevator();
//                    OutTakeMTI.elevator.setTargetPosition(3.5 * OutTakeMTI.STEP);
                    OutTakeMTI.align = true;
                    OutTakeMTI.arm.rotationIndex = 0;
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                })
                .lineToLinearHeading(LeftYellow)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;

                    OutTakeMTI.driverUpdated = true;

                    isInPreloadPhase = false;
                    firstPathAfterPreload = true;
                    OutTakeMTI.arm.rotationIndex = 0;

                })
                .waitSeconds(0.2)
                .lineToLinearHeading(Backdrop)
                .build();


        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    outTake.setToPurplePlacing();
                    isInPreloadPhase = true;
                })
                .lineToLinearHeading(MiddlePurple)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTakeMTI.State.level = 1;
                    outTake.updateElevator();
//                    OutTakeMTI.elevator.setTargetPosition(3.5 * OutTakeMTI.STEP);
                    OutTakeMTI.align = true;
                    OutTakeMTI.arm.rotationIndex = 0;
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                })
                .lineToLinearHeading(MiddleYellow)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;

                    OutTakeMTI.driverUpdated = true;

                    isInPreloadPhase = false;
                    firstPathAfterPreload = true;
                    OutTakeMTI.arm.rotationIndex = 0;

                })
                .lineToLinearHeading(Backdrop)
                .build();
        TrajectorySequence goToStackFromPreloadL = drive.trajectorySequenceBuilder(left.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(55))
                .splineToSplineHeading(BackDropGate, Math.toRadians(90))

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(60))
                .splineToSplineHeading(StackGate, Math.toRadians(90))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                    OutTakeMTI.State.level = 5;
                    ack = false;
                })

                .splineToSplineHeading(Stack, Math.toRadians(90))

                .build();

        TrajectorySequence goToStackFromPreloadM = drive.trajectorySequenceBuilder(middle.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(BackDropGate, Math.toRadians(90))

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(70, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(60))
                .splineToSplineHeading(StackGate, Math.toRadians(90))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                    OutTakeMTI.State.level = 5;
                    ack = false;
                })

                .splineToSplineHeading(Stack, Math.toRadians(90))

                .build();

        TrajectorySequence takeFromStack = drive.trajectorySequenceBuilder(Stack)
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                    pixelsUpdated = false;
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .forward(1)
                .back(1)
                .forward(1)
                .back(1)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    if(!pixelsUpdated) pixelsInStack --;
                })

                .build();
        TrajectorySequence takeFromStack2Line = drive.trajectorySequenceBuilder(Stack)
                .forward(2)
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                    pixelsUpdated = false;
                    pixelsInStack = 5;
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .lineToLinearHeading(Stack2)
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                    pixelsUpdated = false;
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .forward(1)
                .back(1)
                .forward(1)
                .back(1)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    if(!pixelsUpdated) pixelsInStack --;
                })
                .build();
        TrajectorySequence takeFromStack2 = drive.trajectorySequenceBuilder(Stack2)
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                    pixelsUpdated = false;
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .forward(1)
                .back(1)
                .forward(1)
                .back(1)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    if(!pixelsUpdated) pixelsInStack --;
                })
                .build();

        TrajectorySequence goToBackdrop2 = drive.trajectorySequenceBuilder(Stack2)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                })
                .setReversed(false)
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(StackGate, Math.toRadians(-90))

                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(60))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(70, 5, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(BackDropGate, Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    intakeActive = 0;
                })
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 5, DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
                    OutTakeMTI.State.level = 2;
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
                })
                .splineToSplineHeading(new Pose2d(Backdrop.getX(), Backdrop.getY(), Backdrop.getHeading()), Math.toRadians(-100))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                    OutTakeMTI.elevator.setLevel(5);
                })
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    Controls.DropRightAck = false;
                })
                .back(1)
                .waitSeconds(0.2)
                .build();

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(Stack)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                })
                .setReversed(false)
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(60))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(BackDropGate, Math.toRadians(-90))
                .addTemporalMarker(() -> {
                    intakeActive = 0;
                })
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
                    OutTakeMTI.State.level = 2;
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
                })
                .splineToSplineHeading(Backdrop, Math.toRadians(-100))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                    OutTakeMTI.elevator.setLevel(3 + cycle);
                })
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    Controls.DropRightAck = false;
                    cycle++;
                })
                .back(2)
                .waitSeconds(0.2)

                .build();


        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .lineToSplineHeading(new Pose2d(BackDropGate.getX(), BackDropGate.getY(), BackDropGate.getHeading()))

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(55))
                .splineToSplineHeading(StackGate, Math.toRadians(90))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                    ack = false;
                })

                .splineToSplineHeading(Stack, Math.toRadians(90))

                .build();

        TrajectorySequence goToStack2 = drive.trajectorySequenceBuilder(goToBackdrop2.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                    ack = false;
                })

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(55))
                .lineToSplineHeading(BackDropGate)

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(57))
                .splineToSplineHeading(StackGate, Math.toRadians(90))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                })

                .splineToSplineHeading(Stack2, Math.toRadians(90))

                .build();
        while (opModeInInit()){
            outTake.update();
            ControlHub.telemetry.update();
        }
        long time1 = 0;
        ElapsedTime autoTime = new ElapsedTime();

        RedCloseDetectionPipeline.Location location = detector.getLocation();

        switch (detector.getLocation()){
            case MIDDLE:
                drive.followTrajectorySequenceAsync(middle);
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(right);
                break;
            case LEFT:
                drive.followTrajectorySequenceAsync(left);
                break;
        }
        new Thread(camera::closeCameraDevice).start();
//        location = RedCloseDetectionPipeline.Location.RIGHT;
//        drive.followTrajectorySequenceAsync(right);

        while (opModeIsActive()){
            if(order == 1 && autoTime.seconds() > 30 - goToBackdrop.duration()){
                drive.breakFollowing();
                order++;
            }
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();
//            e.update(false, drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY());
            ExpansionHub.ImuYawAngle = Math.toDegrees(drive.getPoseEstimate().getHeading()) - ExpansionHub.beforeReset;
            if((System.currentTimeMillis() - time1) >= 1.0 / 4){
                double Yawn = ExpansionHub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Yawn));
//                ExpansionHub.ImuYawAngle = Yawn - ExpansionHub.beforeReset;
                time1 = System.currentTimeMillis();
            }
            if(!secondStackCycle && pixelsInStack <= 0 && order == 1 && !drive.isBusy()){
                pixelsInStack = 5;
                transtStack = true;
                drive.followTrajectorySequenceAsync(takeFromStack2Line);
                intake.setPixelStackPosition(pixelsInStack);
                secondStackCycle = true;
            }

            if((order == 1 && OutTakeMTI.isFullOfPixels())){
                pixelsInStack --;
                if(pixelsInStack < 0) pixelsInStack = 0;
                drive.breakFollowing();
                order ++;
                pixelsUpdated = true;
            }
            else if(order == 1 && OutTakeMTI.hasAPixel() && !ack){
                pixelsInStack --;
                drive.breakFollowing();
                if(pixelsInStack <= 0 && !secondStackCycle){
                    intake.setPixelStackPosition(pixelsInStack);
                    pixelsInStack = 5;
                    transtStack = true;
                    drive.followTrajectorySequenceAsync(takeFromStack2Line);
                    secondStackCycle = true;
                }
                ack = true;
                pixelsUpdated = true;
            }
            Controls.Intake = false;
            Controls.RevIntake = false;

            if(intakeActive == 1){
                Controls.Intake = true;
            } else if(intakeActive == -1){
                Controls.RevIntake = true;
                Intake.forceOut = true;
            }

//            if(!isInPreloadPhase && !drive.isBusy()){
//                switch (order) {
//                    case 0:
//                        if((autoTime.seconds() < 30 - goToStack.duration() - goToBackdrop.duration() && !secondStackCycle) ||
//                                (autoTime.seconds() < 30 - goToStack2.duration() - goToBackdrop.duration())) {
//                            if(firstPathAfterPreload)
//                                switch (location){
//                                    case MIDDLE:
//                                        drive.followTrajectorySequenceAsync(goToStackFromPreloadM);
//                                        break;
//                                    case LEFT:
//                                        drive.followTrajectorySequenceAsync(goToStackFromPreloadL);
//                                        break;
//                                }
//                            else {
//                                Pose2d pose = drive.getPoseEstimate();
//                                drive.setPoseEstimate(new Pose2d(pose.getX() + 0.3, pose.getY(), pose.getHeading()));
//                                if(secondStackCycle) drive.followTrajectorySequenceAsync(goToStack2);
//                                else drive.followTrajectorySequenceAsync(goToStack);
//                            }
////                            order++;
//                        }
//                        break;
//                    case 1:
//                        if(secondStackCycle) drive.followTrajectorySequenceAsync(takeFromStack2);
//                        else drive.followTrajectorySequenceAsync(takeFromStack);
//                        break;
//                    case 2:
//                        if(autoTime.seconds() > 13)
//                        if(secondStackCycle)
//                            drive.followTrajectorySequenceAsync(goToBackdrop2);
//                        else drive.followTrajectorySequenceAsync(goToBackdrop);
////                        order = 0;
//                    default:
//                        break;
//                }
//            }

            drive.update();

            intake.update_values();
            intake.update();
            outTake.update();
            cn.loop();
            ControlHub.telemetry.addData("pixels in stack", pixelsInStack);
            ControlHub.telemetry.addData("Time Left", 30 - autoTime.seconds());
            ControlHub.telemetry.update();
            if(autoTime.seconds() >= 30.1) requestOpModeStop();
        }
        OutTakeMTI.timeToDrop = 0.3;
        Intake.reversePower = -1;

    }
}
