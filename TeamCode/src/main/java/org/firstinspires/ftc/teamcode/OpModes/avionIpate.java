package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Avion;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;

import java.util.ArrayList;

@TeleOp(name = "avion")
public class avionIpate extends LinearOpMode {
    public Avion a;
    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>()));
        Controls cn = new Controls(gamepad1, gamepad2);

        ControlHub.telemetry = telemetry;

        a = new Avion();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            a.update();
            cn.loop();
            telemetry.update();
        }

    }
}
