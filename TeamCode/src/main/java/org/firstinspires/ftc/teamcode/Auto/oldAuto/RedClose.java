package org.firstinspires.ftc.teamcode.Auto.oldAuto;

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

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.detectionPipelines.RedCloseDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "redClose", preselectTeleOp = ".pipers \uD83C\uDF36️")
@Config
@Disabled
public class RedClose extends LinearOpMode {
    enum State{
        INTAKE,
        OTHER_INTAKE,
        NOT_INTAKE
    }

    private State state = State.NOT_INTAKE;

    public static double middlePurple_x = 21.5, middlePurple_y = -2.5,
            middleYellow_x = 23, middleYellow_y = -31, middleYellow_h = Math.toRadians(-70),
    leftpurple_x = 16, leftpurple_y = 6, leftpurple_h = 0.74,
    leftyellow_x = 29, leftyellow_y = -31, leftyellow_h = Math.toRadians(-70),
            rightpurple_x = 11.6, rightpurple_y = -4, rightpurple_h = 6,
            rightyellow_x = 17, rightyellow_y = -31, rightyellow_h = Math.toRadians(-70),
    transit_x = 5, transit_y = 2, transit_h = Math.toRadians(-90),
    angledStack_x = 22.5, angledStack_y = 76.5, angledStack_h = Math.toRadians(-115);

    public static Pose2d middlePurple = new Pose2d(middlePurple_x, middlePurple_y, 0),
    middleYellow = new Pose2d(middleYellow_x, middleYellow_y, middleYellow_h);
    boolean readSensor = false;
    private TrajectorySequence toAngledStack, otherToAngledStack, park;

    private int stackPos = 0;
    private SampleMecanumDriveCancelable mecanumDrive;
    public static int cycle = 0;
    private final ElapsedTime TIME = new ElapsedTime();
    private OpenCvCamera camera;
    public double IMU_FREQ = 4;

    @Override
    public void runOpMode() throws InterruptedException {

        cycle = 0;

        mecanumDrive = new SampleMecanumDriveCancelable(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        ControlHub ch = new ControlHub(hardwareMap);
        ControlHub.telemetry = telemetry;
        ExpansionHub eh = new ExpansionHub(hardwareMap, mecanumDrive.getLocalizer());
        Controls c = new Controls(gamepad1, gamepad2);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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

        OutTake outTake = new OutTake(hardwareMap);
        Intake intake = new Intake();
        intake.setPixelStackPosition(0);

        OutTake.finalPivotPivotAngle = 200;
        OutTake.finalArmAngle = 230;
        OutTake.intermediarPivot = 130;
        ExpansionHub.extension_length = 6900;
        ExpansionHub.ImuYawAngle = 0;

        TrajectorySequence left = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 0;
                    readSensor = false;
                    Controls.ExtendElevator = true;
                    ExpansionHub.extension_length = 6900;
                    ExpansionHub.ImuYawAngle = 0;
                })
                .lineToLinearHeading(new Pose2d(leftpurple_x, leftpurple_y, leftpurple_h))
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    OutTake.outTakeExtension.MOTION_PROFILED = true;
                    OutTake.State.level = 0;
                    ExpansionHub.extension_length = 0;
                    Controls.ElevatorUp = true;
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                    OutTake.elevatorArm.setArmAngle(OutTake.finalArmAngle);
                    OutTake.elevatorArm.setPivotAngle(OutTake.finalPivotPivotAngle);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    ExpansionHub.extension_length = 6900;
                    ExpansionHub.ImuYawAngle = 90 + Math.toDegrees(leftyellow_h);
                })
                .lineToLinearHeading(new Pose2d(leftyellow_x, leftyellow_y, leftyellow_h))
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(transit_x, transit_y, transit_h), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 10), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 48), -transit_h)
                .splineToSplineHeading(new Pose2d(angledStack_x, angledStack_y, angledStack_h), -angledStack_h - 1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence middle = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    ExpansionHub.extension_length = 6900;
                    ExpansionHub.ImuYawAngle = 0;
                    OutTake.State.level = 0;
                    readSensor = false;
                    Controls.ExtendElevator = true;
                })
                .lineToLinearHeading(middlePurple)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Controls.ElevatorUp = true;
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                    OutTake.elevatorArm.setArmAngle(OutTake.finalArmAngle);
                    OutTake.elevatorArm.setPivotAngle(OutTake.finalPivotPivotAngle);
                    ExpansionHub.ImuYawAngle = 90 + Math.toDegrees(middleYellow.getHeading());
                    ExpansionHub.extension_length = 6900;
                })
                .lineToLinearHeading(middleYellow)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(transit_x, transit_y, transit_h), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 10), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 48), -transit_h)
                .splineToSplineHeading(new Pose2d(angledStack_x, angledStack_y, angledStack_h), -angledStack_h - 1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence right = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 0;
                    readSensor = false;
                    Controls.ExtendElevator = true;
                    ExpansionHub.extension_length = 6900;
                    ExpansionHub.ImuYawAngle = 0;
                })
                .lineToLinearHeading(new Pose2d(rightpurple_x, rightpurple_y, rightpurple_h))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    OutTake.outTakeExtension.MOTION_PROFILED = true;
                    Controls.ElevatorUp = true;
                    ExpansionHub.ImuYawAngle = 90 + Math.toDegrees(leftyellow_h);
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                    OutTake.elevatorArm.setArmAngle(OutTake.finalArmAngle);
                    OutTake.elevatorArm.setPivotAngle(OutTake.finalPivotPivotAngle);
                })
                .lineToLinearHeading(new Pose2d(rightyellow_x, rightyellow_y, rightyellow_h))
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(transit_x, transit_y, transit_h), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 10), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 48), -transit_h)
                .splineToSplineHeading(new Pose2d(angledStack_x, angledStack_y, angledStack_h), -angledStack_h - 1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence angledIntake = mecanumDrive.trajectorySequenceBuilder(left.end())
                .addTemporalMarker(() -> {
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
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, -0.65);
                })
                .waitSeconds(0.3)
                .build();

        TrajectorySequence angledToBackDrop = mecanumDrive.trajectorySequenceBuilder(left.end())
                .addTemporalMarker(() -> {
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(0.3, () -> {
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, -1);
                })
                .addTemporalMarker(1, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(1.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 0);
                })
                .lineToSplineHeading(new Pose2d(transit_x, transit_y + 48, transit_h))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 30), transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y), transit_h)
                .UNSTABLE_addTemporalMarkerOffset(-0.85, () -> {
                    OutTake.State.level = 3;
                    Controls.ExtendElevator = true;
                    readSensor = true;
                })
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(40))
                .splineToSplineHeading(new Pose2d(leftyellow_x-13, leftyellow_y - 1, Math.toRadians(-65)), Math.toRadians(-65))
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    readSensor = false;
                })
                .addTemporalMarker(() -> {
                    if(TIME.seconds() < 22) {
                        if(cycle != 2) mecanumDrive.followTrajectorySequenceAsync(toAngledStack);
                        else mecanumDrive.followTrajectorySequenceAsync(otherToAngledStack);
                    } else mecanumDrive.followTrajectorySequenceAsync(park);
                })
                .build();

        TrajectorySequence otherAngledIntake = mecanumDrive.trajectorySequenceBuilder(new Pose2d(left.end().getX() + 13, left.end().getY(), left.end().getHeading()))
                .addTemporalMarker(() -> {
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
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, -0.65);
                })
                .waitSeconds(0.3)
                .build();

        TrajectorySequence otherAngledToBackDrop = mecanumDrive.trajectorySequenceBuilder(otherAngledIntake.end())
                .addTemporalMarker(() -> {
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(0.3, () -> {
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, -1);
                })
                .addTemporalMarker(1, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(1.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 0);
                })
                .lineToSplineHeading(new Pose2d(transit_x, transit_y + 48, transit_h))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 30), transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y), transit_h)
                .UNSTABLE_addTemporalMarkerOffset(-0.85, () -> {
                    OutTake.State.level = cycle + 1;
                    Controls.ExtendElevator = true;
                    readSensor = true;
                })
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(40))
                .splineToSplineHeading(new Pose2d(leftyellow_x-15, leftyellow_y - 1, Math.toRadians(-55)), Math.toRadians(-55))
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    readSensor = false;
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    mecanumDrive.followTrajectorySequenceAsync(park);
                })
                .build();

        otherToAngledStack  = mecanumDrive.trajectorySequenceBuilder(angledToBackDrop.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(transit_x, transit_y, transit_h), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 10), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 48), -transit_h)
                .splineToSplineHeading(new Pose2d(angledStack_x, angledStack_y, angledStack_h), -angledStack_h - 1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = 4;
                    intake.servo.setAngle(Intake.stackPositions[4]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .lineToLinearHeading(new Pose2d(angledStack_x + 13, angledStack_y, angledStack_h))
                .UNSTABLE_addTemporalMarkerOffset(-0.55, () -> {
                    stackPos = 0;
                    intake.servo.setAngle(Intake.stackPositions[0]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.OTHER_INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        toAngledStack = mecanumDrive.trajectorySequenceBuilder(angledToBackDrop.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(transit_x, transit_y, transit_h))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 10), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y + 48), -transit_h)
                .splineToSplineHeading(new Pose2d(angledStack_x, angledStack_y, angledStack_h), -angledStack_h - 1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = min(stackPos + 2, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M3, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        park = mecanumDrive.trajectorySequenceBuilder(angledToBackDrop.end())
                .lineToLinearHeading(new Pose2d(transit_x, rightyellow_y, Math.toRadians(-90)))
                .build();

        OutTake.leftGripper.update_values();
        OutTake.rightGripper.update_values();

        OutTake.leftGripper.update();
        OutTake.rightGripper.update();

        while(opModeInInit()){
            outTake.update();
            outTake.update_values();
            if(detector.getLocation() == RedCloseDetectionPipeline.Location.LEFT)
                telemetry.addLine("LEFT");
            else if(detector.getLocation() == RedCloseDetectionPipeline.Location.MIDDLE)
                telemetry.addLine("MIDDLE");
            else if(detector.getLocation() == RedCloseDetectionPipeline.Location.RIGHT)
                telemetry.addLine("RIGHT");
            telemetry.update();
        }

        new Thread(() -> {
            camera.closeCameraDevice();
        }).start();

        TIME.reset();
        if(detector.getLocation() == RedCloseDetectionPipeline.Location.LEFT)
            mecanumDrive.followTrajectorySequenceAsync(left);
        else if(detector.getLocation() == RedCloseDetectionPipeline.Location.MIDDLE)
            mecanumDrive.followTrajectorySequenceAsync(middle);
        else if(detector.getLocation() == RedCloseDetectionPipeline.Location.RIGHT)
            mecanumDrive.followTrajectorySequenceAsync(right);
        freq.reset();

        ElapsedTime imuTime = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()){

            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();

            if(readSensor){
                ExpansionHub.ImuYawAngle = 90 + Math.toDegrees(mecanumDrive.getPoseEstimate().getHeading());
                if(ExpansionHub.ImuYawAngle > 180) ExpansionHub.ImuYawAngle -= 360;
                if(ExpansionHub.ImuYawAngle < -180) ExpansionHub.ImuYawAngle += 360;
            }

            if(cycle == 2 && state == State.INTAKE) {
                state = State.OTHER_INTAKE;
            }

            if(state == State.INTAKE){
                    if (OutTake.fullPixel() || TIME.seconds() > 26) {
                        state = null;
                        mecanumDrive.breakFollowing();
                        mecanumDrive.followTrajectorySequenceAsync(angledToBackDrop);

                        cycle++;
                    } else if (!mecanumDrive.isBusy()) {
                        mecanumDrive.followTrajectorySequenceAsync(angledIntake);
                    }
            }

            if(state == State.OTHER_INTAKE) {
                if (OutTake.fullPixel() || TIME.seconds() > 26) {
                    state = null;
                    mecanumDrive.breakFollowing();
                    mecanumDrive.followTrajectorySequenceAsync(otherAngledToBackDrop);

                    cycle++;
                } else if (!mecanumDrive.isBusy()) {
                    mecanumDrive.followTrajectorySequenceAsync(otherAngledIntake);
                }
            }

            mecanumDrive.update();

            if(imuTime.seconds() > 1.0 / IMU_FREQ) {
                imuTime.reset();
                double imuAngle = ExpansionHub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Pose2d pose = mecanumDrive.getPoseEstimate();

                if(imuAngle != 0) mecanumDrive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), imuAngle));
            }

            outTake.update();
            outTake.update_values();
            intake.servo.update();

            outTake.runTelemetry();
            telemetry.addData("angle", ExpansionHub.ImuYawAngle);
            telemetry.addData("freq", 1.0/freq.seconds());

            telemetry.addData("x: ", mecanumDrive.getPoseEstimate().getX());
            telemetry.addData("y: ", mecanumDrive.getPoseEstimate().getY());
            telemetry.addData("h: ", mecanumDrive.getPoseEstimate().getHeading());
            freq.reset();

            telemetry.update();
            c.loop();
        }

        OutTake.finalPivotPivotAngle = 130;
        OutTake.finalArmAngle = 210;
    }
    ElapsedTime freq = new ElapsedTime();
}