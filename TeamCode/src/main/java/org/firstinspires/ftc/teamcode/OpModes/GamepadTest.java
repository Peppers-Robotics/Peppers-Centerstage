package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.AutoGamepad;

@Config
@TeleOp(name = "gamepadTest")
public class GamepadTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        AutoGamepad gamepad = new AutoGamepad(gamepad1);
        int a = 0;

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){

            gamepad.update();

            if(gamepad.wasPressed.b){
                a++;
            }
            if(gamepad.wasReleased.b){
            }

            telemetry.addData("a", a);
            telemetry.update();
        }
    }
}
