package org.firstinspires.ftc.teamcode.OpModes;

import android.net.LinkAddress;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Parts.GoodMecanumDrive;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;

@TeleOp(name = "TuneAutonomousPIDS")
@Config
public class TuneAutoPIDS extends LinearOpMode {
    public static Pose2d hold = new Pose2d(0, 0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        ExpansionHub eh = new ExpansionHub(hardwareMap, null);
        ControlHub ch = new ControlHub(hardwareMap);
        ControlHub.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GoodMecanumDrive drive = new GoodMecanumDrive();
        drive.follow = true;
        drive.setTargetPosition(hold, hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            drive.update(0, 0, 0);
            ControlHub.telemetry.update();
        }
    }
}
