package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.LinearGradient;

import androidx.core.location.GnssStatusCompat;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@TeleOp(name = "servoTestWithDelay")
public class delayServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap, null);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ControlHub.telemetry = telemetry;

        AutoServo s = new AutoServo(SERVO_PORTS.S3, 50/355.f, true, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);

        waitForStart();
        s.setAngle(100, 500);
        s.setAngle(40, 1200);
        s.setAngle(20, 5000);

        while (opModeIsActive()){

            telemetry.addData("angle", s.getAngle());
            telemetry.update();
            s.update();
        }

    }
}
