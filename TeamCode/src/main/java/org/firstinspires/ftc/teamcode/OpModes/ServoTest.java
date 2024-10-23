package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Parts.MotionProfile;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.util.ArrayList;

@TeleOp(name = "servoTest")
@Config
public class ServoTest extends LinearOpMode {
    public static double position = 0, angle = 0, maxAngle = 355;
    public static boolean Chub = false, Isangle = false;
    public static Servo.Direction dir = Servo.Direction.FORWARD;
    public static SERVO_PORTS port = SERVO_PORTS.S0;
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);


        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub h = new ExpansionHub(hardwareMap, new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>()));

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            if (Chub) {
                ControlHub.setServoDirection(port, dir);
                ControlHub.setServoPosition(port, Isangle ? angle / maxAngle : position);
            } else {
                ExpansionHub.setServoDirection(port, dir);
                ExpansionHub.setServoPosition(port, Isangle ? angle / maxAngle : position);
            }

        }
    }
}
