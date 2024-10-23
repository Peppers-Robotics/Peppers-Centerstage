package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.BetterColorRangeSensor;

@TeleOp(name = "digitalTest")
public class DigitalTest extends LinearOpMode {
    private void a(int x){
        x = (byte)x;
        while (x > 0){
            telemetry.addData("a", x % 2);
            x /= 2;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        BetterColorRangeSensor sensorL = hardwareMap.get(BetterColorRangeSensor.class, "leftSensor");
        BetterColorRangeSensor sensorR = hardwareMap.get(BetterColorRangeSensor.class, "rightSensor");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("right value", sensorR.getProximityDistance());
            telemetry.addData("left value",  sensorL.getProximityDistance());

            telemetry.update();
        }
    }
}
