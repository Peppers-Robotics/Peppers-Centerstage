package org.firstinspires.ftc.teamcode.Auto;

import android.os.Environment;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.detectionPipelines.BlueCloseDetectionPipeline;
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

@Config
@Autonomous(name = "RedClose", preselectTeleOp = ".pipers \uD83C\uDF36", group = "auto")
public class RedClose extends LinearOpMode {
    SampleMecanumDriveCancelable drive;
    OutTakeMTI outTake;
    Intake intake;
    public static Pose2d
            MiddlePurple = new Pose2d(20, 0, 0),
            MiddleYellow = new Pose2d(18.7, -28, Math.toRadians(286)),

    LeftPurple = new Pose2d(9.5, -9, Math.toRadians(-357)),
            LeftYellow = new Pose2d(12.5, -28, Math.toRadians(-76)),

    RightPurple = new Pose2d(15.5, 4, Math.toRadians(45)),
            RightYellow = new Pose2d(20.5, -26.5, Math.toRadians(-60))
                    ;

    public static Pose2d
            TrussToStack     = new Pose2d(4, 45, -Math.PI/2.f),
            Stack            = new Pose2d(23.5, 75, -Math.toRadians(100)),
            Stack2           = new Pose2d(35, 75, -Math.toRadians(100)),
            BackBoardToTruss = new Pose2d(4, 9, -Math.PI/2.f),
            Backdrop         = new Pose2d(12, -28.5, -Math.toRadians(70)),
            TrussToStack_s     = new Pose2d(4, 45, -Math.PI/2.f),
            BackBoardToTruss_s = new Pose2d(4, 9, -Math.PI/2.f)

                    ;
    int pixelsInStack = 5;
    boolean followNext = false;
    int queue = 0;

    boolean ack = false;

    int intakeActive = 0;
    private boolean isInPreloadPhase = false, firstPathAfterPreload = true;
    private boolean pixelsUpdated = false, secondStack = false, wasATryLast = false, isAtStack = false;
    @Override
    public void runOpMode() throws InterruptedException {

        secondStack = false;
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        RedCloseDetectionPipeline detector = new RedCloseDetectionPipeline(telemetry, false);

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


        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, drive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);
        ControlHub.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ExpansionHub.setInitialBackdropAngleRelativeToBot(-90);

        outTake = new OutTakeMTI();
        intake = new Intake();
        Intake.reversePower = -1;

        OutTakeMTI.timeToDrop = 0.25;

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTakeMTI.arm.rotationIndex = 1;
                    outTake.setToPurplePlacing();
                    isInPreloadPhase = true;
                })
                .lineToLinearHeading(LeftPurple)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                    OutTakeMTI.elevator.setTargetPosition(OutTakeMTI.STEP * 3.5);
                    OutTakeMTI.arm.rotationIndex = 0;
                    OutTakeMTI.align = true;
                })
                .lineToLinearHeading(LeftYellow)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                })
                .addTemporalMarker(() -> {
                    OutTakeMTI.driverUpdated = true;
                })
                .addTemporalMarker(() -> {
                    isInPreloadPhase = false;
                    firstPathAfterPreload = true;
                })
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTakeMTI.arm.rotationIndex = 1;
                    outTake.setToPurplePlacing();
                    isInPreloadPhase = true;
                })
                .lineToLinearHeading(RightPurple)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    OutTakeMTI.elevator.setTargetPosition(OutTakeMTI.STEP * 3.5);
                    OutTakeMTI.align = true;
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                    OutTakeMTI.arm.rotationIndex = 0;
                })
//                .turn(Math.toRadians(-60))
                .lineToLinearHeading(RightYellow)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                    OutTakeMTI.driverUpdated = true;
                    isInPreloadPhase = false;
                    firstPathAfterPreload = true;
                })
                .build();

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    outTake.setToPurplePlacing();
                    isInPreloadPhase = true;
                    OutTakeMTI.arm.rotationIndex = 0;
                })
                .lineToLinearHeading(MiddlePurple)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    OutTakeMTI.State.level = 3;
                    OutTakeMTI.elevator.setTargetPosition(3.5 * OutTakeMTI.STEP);
                    OutTakeMTI.align = true;
                    OutTakeMTI.arm.rotationIndex = 0;
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                })
//                .turn(Math.toRadians(-60))
                .lineToLinearHeading(MiddleYellow)
                .waitSeconds(0.05)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                })
                .waitSeconds(0.05)
                .addTemporalMarker(() -> {
                    OutTakeMTI.driverUpdated = true;
                })
                .addTemporalMarker(() -> {
                    isInPreloadPhase = false;
                    firstPathAfterPreload = true;
                    OutTakeMTI.arm.rotationIndex = 0;
                })
                .build();

        ack = false;

        TrajectorySequence goToStackFromPreloadL = drive.trajectorySequenceBuilder(left.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(40))
                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                    ack = false;
                })
                .splineToSplineHeading(TrussToStack, Math.toRadians(90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(Stack, Math.toRadians(45))
                .resetAccelConstraint()
                .addTemporalMarker(() -> {
                    isAtStack = true;
                })
                .build();

        TrajectorySequence goToStackFromPreloadR = drive.trajectorySequenceBuilder(right.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(40))
                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                    ack = false;
                })
                .splineToSplineHeading(TrussToStack, Math.toRadians(90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(Stack, Math.toRadians(45))
                .resetAccelConstraint()
                .addTemporalMarker(() -> {
                    isAtStack = true;
                })
                .build();

        TrajectorySequence goToStackFromPreloadM = drive.trajectorySequenceBuilder(middle.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(45))
                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                    ack = false;
                })
                .splineToSplineHeading(TrussToStack, Math.toRadians(90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(Stack, Math.toRadians(50))
                .resetAccelConstraint()
                .addTemporalMarker(() -> {
                    isAtStack = true;
                })
                .build();



        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(Backdrop)
                .setReversed(true)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                })
                .splineToSplineHeading(TrussToStack, Math.toRadians(90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .addTemporalMarker(() -> {
                    ack = false;
                })

                .splineToSplineHeading(Stack, Math.toRadians(50))
                .resetAccelConstraint()
                .addTemporalMarker(() -> {
                    isAtStack = true;
                })
                .build();



        TrajectorySequence takePixels = drive.trajectorySequenceBuilder(goToStack.end())
                .addTemporalMarker(() -> {
                    wasATryLast = false;
                    intakeActive = 1;
                    pixelsUpdated = false;
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .forward(1)
                .waitSeconds(0.1)
                .back(1)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    if(!pixelsUpdated) pixelsInStack --;
                    if(pixelsInStack == 1) wasATryLast = true;
                })
                .build();

        TrajectorySequence takeFromSecondStack = drive.trajectorySequenceBuilder(takePixels.end())
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .lineToLinearHeading(Stack2)
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                    pixelsUpdated = false;
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .forward(1)
                .waitSeconds(0.1)
                .back(1)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    if(!pixelsUpdated) pixelsInStack --;
                })
                .build();

        TrajectorySequence takeFromSecondStackWithoutLine = drive.trajectorySequenceBuilder(takeFromSecondStack.end())
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                    pixelsUpdated = false;
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .forward(1)
                .back(1)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    if(!pixelsUpdated) pixelsInStack --;
                })
                .build();

        TrajectorySequence goToBackDropFromSecondStack = drive.trajectorySequenceBuilder(takeFromSecondStack.end())
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    isAtStack = false;
                })
                .setReversed(false)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .strafeRight(2)
                .splineToSplineHeading(TrussToStack_s, Math.toRadians(-90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(BackBoardToTruss_s.getX(), BackBoardToTruss_s.getY()), Math.toRadians(-90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    intakeActive = 0;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
                    OutTakeMTI.State.level = 1;
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
//                    distanceSensorMesh = true;
                })
                .splineToSplineHeading(Backdrop, Math.toRadians(-55))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                    OutTakeMTI.State.level = 4;
                    outTake.updateElevator();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.01, () -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    Controls.DropRightAck = false;
                })
                .build();

        TrajectorySequence goToBackDrop = drive.trajectorySequenceBuilder(takePixels.end())
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    isAtStack = false;
                })
                .setReversed(false)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(TrussToStack_s, Math.toRadians(-90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(BackBoardToTruss_s.getX(), BackBoardToTruss_s.getY()), Math.toRadians(-90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 2*Math.PI, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    intakeActive = 0;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
                    OutTakeMTI.State.level = 1;
                    OutTakeMTI.arm.rotationIndex = 3;
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
//                    distanceSensorMesh = true;
                })
                .splineToSplineHeading(Backdrop, Math.toRadians(-55))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                    OutTakeMTI.State.level = 3;
                    outTake.updateElevator();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.01, () -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    Controls.DropRightAck = false;
                    followNext = true;
                })
                .build();

        RedCloseDetectionPipeline.Location location;

        while (opModeInInit()){
            location = detector.getLocation();
            if(location != null)
                telemetry.addData("case", location.toString());
            outTake.update();
            telemetry.update();
        }

        new Thread(() -> {
            camera.closeCameraDevice();
        }).start();

        location = detector.getLocation();

        switch (location){
            case RIGHT:
                drive.followTrajectorySequenceAsync(left);
                break;
            case MIDDLE:
                drive.followTrajectorySequenceAsync(middle);
                break;
            case LEFT:
                drive.followTrajectorySequenceAsync(right);
                break;
        }


        int order = 0;
        long time1 = System.currentTimeMillis();
        ElapsedTime autoTime = new ElapsedTime();
        boolean noTime = false;

        while(opModeIsActive()){
            if(Math.abs(drive.getLastError().getX()) > 15 || Math.abs(drive.getLastError().getY()) > 15 || autoTime.seconds() > 30.5){
                requestOpModeStop();
            }
            if(drive.isBusy() && autoTime.seconds() >= 30 - goToBackDrop.duration() - 1.25 && order == 1 && !noTime){
                drive.breakFollowing();
                noTime = true;
                order = 2;
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
            if(order == 1 && (pixelsInStack == 0 || (!drive.isBusy() && pixelsInStack == 1)) && isAtStack && !noTime){
                secondStack = true;
                pixelsInStack = 1;
                drive.breakFollowing();
                drive.followTrajectorySequenceAsync(takeFromSecondStack);
                pixelsUpdated = true;
                ack = true;
            } else if((order == 1 && OutTakeMTI.isFullOfPixels()) && isAtStack && !noTime){
                pixelsInStack --;
                if(pixelsInStack < 0) pixelsInStack = 0;
                drive.breakFollowing();
                order ++;
                pixelsUpdated = true;
            }
            else if(order == 1 && OutTakeMTI.hasAPixel() && !ack && isAtStack && !noTime){
                pixelsInStack --;
                drive.breakFollowing();
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

            if(!isInPreloadPhase && (!drive.isBusy() || followNext)){
                followNext = false;
                switch (order) {
                    case 0:
                        if(autoTime.seconds() <= 30 - goToStack.duration() - 1.25 - goToBackDrop.duration()) {
                            if (firstPathAfterPreload) {
                                switch (location){
                                    case RIGHT:
                                        drive.followTrajectorySequenceAsync(goToStackFromPreloadL);
                                        break;
                                    case MIDDLE:
                                        drive.followTrajectorySequenceAsync(goToStackFromPreloadM);
                                        break;
                                    case LEFT:
                                        drive.followTrajectorySequenceAsync(goToStackFromPreloadR);
                                        break;
                                }
                            }
                            else drive.followTrajectorySequenceAsync(goToStack);
                            order++;
                            Pose2d pose = drive.getPoseEstimate();
                            drive.setPoseEstimate(new Pose2d(pose.getX() + 0.1, pose.getY(), pose.getHeading()));
                        }
                        break;
                    case 1:
                        if(secondStack)
                            drive.followTrajectorySequenceAsync(takeFromSecondStackWithoutLine);
                        else drive.followTrajectorySequenceAsync(takePixels);
                        break;
                    case 2:
                        if(secondStack)
                            drive.followTrajectorySequenceAsync(goToBackDropFromSecondStack);
                        else drive.followTrajectorySequenceAsync(goToBackDrop);
                        order = 0;
                    default:
                        break;
                }
            }

            drive.update();

            intake.update_values();
            intake.update();
            outTake.update();
            cn.loop();
            ControlHub.telemetry.addData("pixels in stack", pixelsInStack);
            ControlHub.telemetry.update();
        }
        OutTakeMTI.timeToDrop = 0.3;
        Intake.reversePower = -1;
    }
}