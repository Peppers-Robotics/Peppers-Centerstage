package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.Parts.MotionProfile;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.util.ArrayList;

@TeleOp(name = "motionProfileServoTest")
@Config
public class motionProfileServoTuning extends LinearOpMode {
    public static double acceleration = 0.4, maxVelo = 1;
    public static SERVO_PORTS port = SERVO_PORTS.S5;
    public static Hubs hub = Hubs.CONTROL_HUB;
    public static AutoServo.TYPE type = AutoServo.TYPE.AXON;
    public static boolean reversed = true, startTest = false;
    public static double targetAngle = 90;
    public double pos = 0;
    public static AutoServo servo;
    public static MotionProfile motionProfile = new MotionProfile(maxVelo, acceleration);
    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>()));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ControlHub.telemetry = telemetry;
        servo = new AutoServo(port, 0, reversed, hub, type);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            if(motionProfile.motionEnded() && servo.getAngle() != targetAngle) {
                motionProfile.startMotion(servo.getAngle(), targetAngle);
            }

            servo.update();
            motionProfile.update();

            servo.setAngle(motionProfile.getPosition());

            telemetry.addData("position", motionProfile.getPosition());
            telemetry.update();
        }
    }
}
