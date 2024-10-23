package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;

@Config
@TeleOp(name = "encoderTest")
public class encoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ExpansionHub eh = new ExpansionHub(hardwareMap, null);
        ControlHub ch = new ControlHub(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ControlHub.telemetry = telemetry;

        waitForStart();

        while(opModeIsActive()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();
            for(int i = 0; i < 4; i++){
                ControlHub.encoder[i].read = false;
                ExpansionHub.encoder[i].read = false;
            }
            telemetry.addData("eM0: ", ExpansionHub.getEncoderPosition(ENCODER_PORTS.E0));
            telemetry.addData("eM1: ", ExpansionHub.getEncoderPosition(ENCODER_PORTS.E1));
            telemetry.addData("eM2: ", ExpansionHub.getEncoderPosition(ENCODER_PORTS.E2));
            telemetry.addData("eM3: ", ExpansionHub.getEncoderPosition(ENCODER_PORTS.E3));

            telemetry.addData("cM0: ", ControlHub.getEncoderPosition(ENCODER_PORTS.E0));
            telemetry.addData("cM1: ", ControlHub.getEncoderPosition(ENCODER_PORTS.E1));
            telemetry.addData("cM2: ", ControlHub.getEncoderPosition(ENCODER_PORTS.E2));
            telemetry.addData("cM3: ", ControlHub.getEncoderPosition(ENCODER_PORTS.E3));
            telemetry.update();
        }
    }
}
