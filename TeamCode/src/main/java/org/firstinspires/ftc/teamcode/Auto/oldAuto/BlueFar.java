package org.firstinspires.ftc.teamcode.Auto.oldAuto;

import static org.firstinspires.ftc.teamcode.detectionPipelines.BlueFarDetectionPipeline.Location.LEFT;
import static org.firstinspires.ftc.teamcode.detectionPipelines.BlueFarDetectionPipeline.Location.MIDDLE;
import static org.firstinspires.ftc.teamcode.detectionPipelines.BlueFarDetectionPipeline.Location.RIGHT;
import static org.firstinspires.ftc.teamcode.Parts.OutTake.State.step;
import static java.lang.Math.abs;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.detectionPipelines.BlueFarDetectionPipeline;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.AprilTagDetector;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.AprilTagMath;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

// purple
// 38, 11, Math.toRadians(270)
// stack
// 50, 18, Math.toRadians(270)

@Autonomous(name = "blueFar", preselectTeleOp = ".pipers \uD83C\uDF36️")
@Config
@Disabled
public class BlueFar extends LinearOpMode {
    enum State{
        INTAKE,
        NOT_INTAKE,
        GET_IN,
        GET_OUT,
        RELOCALIZATION
    }
    enum CASE {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private CASE caz = CASE.LEFT;

    private State state = State.NOT_INTAKE;
    private boolean relocalize = false;
    private int cycle = 0;
    private ElapsedTime get_out_timer = new ElapsedTime();
    private boolean align = true;

    public static double
            stack_x = 49, stack_y = -18.5, stack_h = -Math.toRadians(-90),
            park_x = 49, park_y = 92, park_h = -Math.toRadians(-90),
            middlepurple_x = 36, middlepurple_y = -14.5, middlepurple_h = -Math.toRadians(-90),
            leftpurple_x = 10.5, leftpurple_y = -5, leftpurple_h = -Math.toRadians(0),
            rightpurple_x = 16, rightpurple_y = 3, rightpurple_h = -Math.toRadians(300),
    middleyellow_x = 32, middleyellow_y = 88.5, middleyellow_h = -Math.toRadians(-110);

    private int stackPos = 0;
    private SampleMecanumDriveCancelable mecanumDrive;
    private ElapsedTime TIME = new ElapsedTime();
    private OpenCvCamera camera;
    public double IMU_FREQ = 4;
    private boolean aprilTagInited = false;

    public TrajectorySequence left, middle, right;
    double TRAJ_TIME = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        mecanumDrive = new SampleMecanumDriveCancelable(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        ControlHub ch = new ControlHub(hardwareMap);
        ControlHub.telemetry = telemetry;
        ExpansionHub eh = new ExpansionHub(hardwareMap, mecanumDrive.getLocalizer());
        Controls c = new Controls(gamepad1, gamepad2);

        AprilTagDetector.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        BlueFarDetectionPipeline detector = new BlueFarDetectionPipeline(telemetry, true);

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

        OutTake outTake = new OutTake(hardwareMap);
        Intake intake = new Intake();
        intake.setPixelStackPosition(0);

        OutTake.leftGripper.update_values();
        OutTake.rightGripper.update_values();

        OutTake.leftGripper.update();
        OutTake.rightGripper.update();

        OutTake.finalPivotPivotAngle = 200;
        OutTake.finalArmAngle = 230;
        OutTake.intermediarPivot = 130;
        ExpansionHub.extension_length = 6900;
        ExpansionHub.ImuYawAngle = 0;

        middle = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 0;
                    Controls.ExtendElevator = true;
                    ExpansionHub.ImuYawAngle = 0;
                    align = false;
                })
                .lineToLinearHeading(new Pose2d(middlepurple_x, middlepurple_y, middlepurple_h))
                .waitSeconds(0.01)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                })
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                    align = true;
                })
                .lineToLinearHeading(new Pose2d(stack_x, stack_y, stack_h))
                .UNSTABLE_addTemporalMarkerOffset(-0.15, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                    state = State.INTAKE;
                })
                .build();

        left = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 0;
                    Controls.ExtendElevator = true;
                    ExpansionHub.extension_length = 6900;
                    align = false;
                    ExpansionHub.ImuYawAngle = 0;
                })
                .lineToLinearHeading(new Pose2d(leftpurple_x, leftpurple_y, leftpurple_h))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    Controls.DropRight = true;
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(leftpurple_x, 6, leftpurple_h))
                .lineToLinearHeading(new Pose2d(stack_x - 5, 0, 0))
                .lineToLinearHeading(new Pose2d(stack_x, stack_y, stack_h))
                .addTemporalMarker(() -> {
                    align = true;
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.15, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                    state = State.INTAKE;
                })
                .build();

        right = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 1;
                    Controls.ExtendElevator = true;
                })
                .addTemporalMarker(0.7, () -> {
                    ExpansionHub.extension_length = 6900;
                    align = false;
                    ExpansionHub.ImuYawAngle = 0;
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 2, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(30))
                .splineToSplineHeading(new Pose2d(rightpurple_x, rightpurple_y, rightpurple_h), rightpurple_h)
                .addTemporalMarker(() -> {
                    OutTake.elevator.setInstantPosition(0);
                    OutTake.State.level--;
                    outTake.update_values();
                    outTake.update();
                    outTake.update_values();
                    outTake.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    Controls.DropRight = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    align = true;
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                })
                .lineToLinearHeading(new Pose2d(stack_x, stack_y + 3, stack_h))
                .addTemporalMarker(() -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                    state = State.INTAKE;
                })
                .lineToLinearHeading(new Pose2d(stack_x, stack_y, stack_h))
                .build();

        TrajectorySequence preload = mecanumDrive.trajectorySequenceBuilder(middle.end())
                .addTemporalMarker(() -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .forward(3)
                .back(3)
                .forward(3)
                .back(3)
                .addTemporalMarker(() -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, -0.8);
                })
                .waitSeconds(0.3)
                .build();

        TrajectorySequence intakein = mecanumDrive.trajectorySequenceBuilder(middle.end())
                .addTemporalMarker(() -> {
                    if(cycle == 2) stackPos = 4;
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .forward(3)
                .back(3)
                .forward(3)
                .back(3)
                .addTemporalMarker(() -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, -0.8);
                })
                .waitSeconds(0.3)
                .build();

        TrajectorySequence yellowLeftStart = mecanumDrive.trajectorySequenceBuilder(new Pose2d(stack_x, stack_y, stack_h))
                .splineToConstantHeading(new Vector2d(stack_x, stack_y + 60), stack_h)
                .splineToSplineHeading(new Pose2d(middleyellow_x+8, middleyellow_y - 14, middleyellow_h), middleyellow_h + 0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    OutTake.State.level = 1.7;
                    Controls.ExtendElevator = true;
                    outTake.MANUAL_EXTENSION = true;
                })
                .addTemporalMarker(() -> state = State.RELOCALIZATION)
                .build();

        TrajectorySequence yellowLeftPlace = mecanumDrive.trajectorySequenceBuilder(yellowLeftStart.end())
                .addTemporalMarker(0.1,  () -> {
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .lineToLinearHeading(new Pose2d(middleyellow_x+8, middleyellow_y - 1, middleyellow_h))
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    Controls.DropLeft = true;
                    OutTake.elevator.update_values();
                    OutTake.elevator.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.outTakeExtension.deactivate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(middleyellow_x+4.5, middleyellow_y, middleyellow_h))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    OutTake.State.level = 3;
                    OutTake.elevator.setInstantPosition(3 * step);
                    OutTake.elevator.update();
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    outTake.MANUAL_EXTENSION = false;
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(new Pose2d(stack_x - 2, stack_y + 80, stack_h), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x - 2, stack_y + 79.9), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y + 30), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y), -stack_h)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence yellowMiddleStart = mecanumDrive.trajectorySequenceBuilder(new Pose2d(stack_x, stack_y, stack_h))
                .splineToConstantHeading(new Vector2d(stack_x, stack_y + 60), stack_h)
                .splineToSplineHeading(new Pose2d(middleyellow_x+2, middleyellow_y - 14, middleyellow_h), middleyellow_h + 0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    OutTake.State.level = 1.7;
                    Controls.ExtendElevator = true;
                    outTake.MANUAL_EXTENSION = true;
                })
                .addTemporalMarker(() -> state = State.RELOCALIZATION)
                .build();

        TrajectorySequence yellowMiddlePlace = mecanumDrive.trajectorySequenceBuilder(yellowLeftStart.end())
                .addTemporalMarker(0.1,  () -> {
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .lineToLinearHeading(new Pose2d(middleyellow_x - 2, middleyellow_y, middleyellow_h))
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    Controls.DropLeft = true;
                    OutTake.State.level = 1.5;
                    OutTake.elevator.setInstantPosition(OutTake.State.level * step);
                    OutTake.elevator.update_values();
                    OutTake.elevator.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    OutTake.outTakeExtension.deactivate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    outTake.update();
                    outTake.update_values();
                    outTake.update();
                    outTake.update_values();
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(middleyellow_x + 4.5, middleyellow_y, middleyellow_h))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    OutTake.elevator.setInstantPosition(3 * step);
                    OutTake.State.level = 3;
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    outTake.MANUAL_EXTENSION = false;
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(new Pose2d(stack_x - 2, stack_y + 80, stack_h), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x - 2, stack_y + 79.9), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y + 30), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y), -stack_h)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence yellowRightStart = mecanumDrive.trajectorySequenceBuilder(new Pose2d(stack_x, stack_y, stack_h))
                .splineToConstantHeading(new Vector2d(stack_x, stack_y + 60), stack_h)
                .splineToSplineHeading(new Pose2d(middleyellow_x, middleyellow_y - 14, Math.toRadians(120)), Math.toRadians(120) + 0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    OutTake.State.level = 2;
                    Controls.ExtendElevator = true;
                    outTake.MANUAL_EXTENSION = true;
                })
                .addTemporalMarker(() -> state = State.RELOCALIZATION)
                .build();

        TrajectorySequence yellowRightPlace = mecanumDrive.trajectorySequenceBuilder(yellowLeftStart.end())
                .addTemporalMarker(0.1,  () -> {
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .lineToLinearHeading(new Pose2d(middleyellow_x - 9, middleyellow_y, middleyellow_h))
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    Controls.DropLeft = true;
                    OutTake.State.level = 1.5;
                    OutTake.elevator.setInstantPosition(OutTake.State.level * step);
                    OutTake.elevator.update_values();
                    OutTake.elevator.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.outTakeExtension.deactivate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(middleyellow_x + 4, middleyellow_y, middleyellow_h))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    OutTake.elevator.setInstantPosition(3 * step);
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    outTake.MANUAL_EXTENSION = false;
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(new Pose2d(stack_x - 2, stack_y + 80, stack_h), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x - 2, stack_y + 79.9), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y + 30), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y), -stack_h)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence toBackDrop = mecanumDrive.trajectorySequenceBuilder(new Pose2d(stack_x, stack_y, stack_h))
                .splineToConstantHeading(new Vector2d(stack_x, stack_y + 80), stack_h)
                .splineToSplineHeading(new Pose2d(stack_x - 13, middleyellow_y + 1, Math.toRadians(120)), Math.toRadians(120))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    OutTake.State.level = cycle + 1;
                    ExpansionHub.extension_length = 6900;
                    Controls.ExtendElevator = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    Controls.DropLeft = true;
                    Controls.DropRight = true;
                })
                .addTemporalMarker(() -> {
                    if(cycle == 3 || TIME.seconds() > 23) {
                        mecanumDrive.breakFollowing();
                        mecanumDrive.followTrajectorySequenceAsync(mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate()).lineToLinearHeading(new Pose2d(park_x, park_y, park_h)).build());
                        TRAJ_TIME = TIME.seconds();
                    }
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(stack_x - 2, stack_y + 80, stack_h), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x - 2, stack_y + 79.9), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y + 30), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x-2, stack_y), -stack_h)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.3)
                .build();

        while(opModeInInit()){
            outTake.update();
            outTake.update_values();
            if(detector.getLocation() == LEFT)
                telemetry.addLine("LEFT");
            else if(detector.getLocation() == MIDDLE)
                telemetry.addLine("MIDDLE");
            else if(detector.getLocation() == RIGHT)
                telemetry.addLine("RIGHT");

            telemetry.addData("Global Shutter Opened: ", AprilTagDetector.OPENED);
            telemetry.addData("fps: ", AprilTagDetector.camera.getPipelineTimeMs());
            // Robot.log.vv("fps: ", Integer.toString(AprilTagDetector.camera.getPipelineTimeMs()));
            telemetry.update();
        }

        new Thread(() -> {
            if(AprilTagDetector.camera.getPipelineTimeMs() > 0) {
                camera.stopStreaming();
                camera.closeCameraDevice();
            } else {
                AprilTagDetector.setCamera((OpenCvWebcam) camera);
            }

            FtcDashboard.getInstance().startCameraStream(AprilTagDetector.camera, 30);
        }).start();

        TIME.reset();
        if(detector.getLocation() == LEFT)
            mecanumDrive.followTrajectorySequenceAsync(right);
        else if(detector.getLocation() == MIDDLE)
            mecanumDrive.followTrajectorySequenceAsync(middle);
        else if(detector.getLocation() == RIGHT)
            mecanumDrive.followTrajectorySequenceAsync(left);
        freq.reset();

        ElapsedTime imuTime = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()){

            ExpansionHub.ExpansionHubModule.clearBulkCache();
            ControlHub.ControlHubModule.clearBulkCache();

            mecanumDrive.update();
            if(align) ExpansionHub.ImuYawAngle = -90 + Math.toDegrees(mecanumDrive.getPoseEstimate().getHeading());

            if(imuTime.seconds() > 1.0 / IMU_FREQ) {
                imuTime.reset();
                double imuAngle = ExpansionHub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Pose2d pose = mecanumDrive.getPoseEstimate();

                if(imuAngle != 0) mecanumDrive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), imuAngle));
            }

            if(state == State.GET_IN) {
                if(!OutTake.fullPixel()) get_out_timer.reset();
                if(OutTake.fullPixel() && get_out_timer.seconds() > 0.4 ) {
                    state = State.GET_OUT;
                    intake.servo.setAngle(60);
                    get_out_timer.reset();
                } else {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                }

            } else if(state == State.GET_OUT) {
                if(get_out_timer.seconds() > 0.6) {
                    state = State.NOT_INTAKE;
                    intake.servo.setAngle(60);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 0);
                } else {
                    intake.servo.setAngle(60);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, -1);
                }
            }

            if(state == State.INTAKE) {

                if(OutTake.fullPixel() || TIME.seconds() > 25) {
                    state = State.GET_IN;
                    get_out_timer.reset();
                    mecanumDrive.breakFollowing();
                    if(cycle == 0) {
                        if(detector.getLocation() == LEFT)
                            mecanumDrive.followTrajectorySequenceAsync(yellowRightStart);
                        else if(detector.getLocation() == MIDDLE)
                            mecanumDrive.followTrajectorySequenceAsync(yellowMiddleStart);
                        else
                            mecanumDrive.followTrajectorySequenceAsync(yellowLeftStart);
                    }
                    else
                        mecanumDrive.followTrajectorySequenceAsync(toBackDrop);

                    cycle++;
                } else if(!mecanumDrive.isBusy()) {
                    if(cycle == 0)
                        mecanumDrive.followTrajectorySequenceAsync(preload);
                    else
                        mecanumDrive.followTrajectorySequenceAsync(intakein);
                }

            }

            if(state == State.RELOCALIZATION) {

                    Pose2d robotPose = mecanumDrive.getPoseEstimate();

                    telemetry.addLine("Robot Pose from odo");
                    telemetry.addData("x:", robotPose.getX());
                    telemetry.addData("y:", robotPose.getY());
                    telemetry.addData("heading:", robotPose.getHeading());

                    AprilTagDetection[] detections = AprilTagDetector.getDetections();
                    AprilTagDetection desiredDetection = null;
                    int desiredId = (detector.getLocation() == LEFT ? 1 : (detector.getLocation() == MIDDLE ? 2 : 3));

                    for (AprilTagDetection detection : detections)
                        if (detection.id == desiredId) desiredDetection = detection;
                    if (desiredDetection != null) {
                        Pose2d aprilPose = AprilTagMath.poseFromTag(robotPose, desiredDetection);

                        telemetry.addLine("Robot Pose from apriltag");
                        telemetry.addData("x:", aprilPose.getX());
                        telemetry.addData("y:", aprilPose.getY());
                        telemetry.addData("heading:", aprilPose.getHeading());

                        new Thread(() -> {
                            AprilTagDetector.camera.closeCameraDevice();
                        }).start();

                        if(AprilTagDetector.OPENED) mecanumDrive.setPoseEstimate(new Pose2d(aprilPose.getX(), robotPose.getY(), robotPose.getHeading()));
                        if (detector.getLocation() == LEFT)
                            mecanumDrive.followTrajectorySequenceAsync(yellowRightPlace);
                        else if (detector.getLocation() == MIDDLE)
                            mecanumDrive.followTrajectorySequenceAsync(yellowMiddlePlace);
                        else
                            mecanumDrive.followTrajectorySequenceAsync(yellowLeftPlace);

                        state = State.NOT_INTAKE;
                    }
            }

            outTake.update();
            outTake.update_values();
            intake.servo.update();

            freq.reset();

            telemetry.update();
            c.loop();
        }

        OutTake.finalPivotPivotAngle = 130;
        OutTake.finalArmAngle = 210;
    }
    ElapsedTime freq = new ElapsedTime();
}
