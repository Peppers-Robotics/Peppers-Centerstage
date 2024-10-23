package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.LinearGradient;
import android.graphics.Path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.AutoUtils.Paths;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.GoodMecanumDrive;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;

@Config
@Autonomous(name = "blueClosePTP")
@Disabled
public class BlueClosePTP extends LinearOpMode {
    public enum State{
        TAKE_FROM_STACK,
        TO_STACK,
        TO_BACKDROP,
        PRELOAD;
    }
    int cycle = 0;
    public static GoodMecanumDrive drive;
    public static OutTakeMTI outtake;
    public static Intake intake;
    public State state;

    public static Paths currentPath;

    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap, null);
        Controls cn = new Controls(gamepad1, gamepad2);
        ControlHub.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new GoodMecanumDrive();
        drive.follow = true;
        drive.setTargetPosition(new Pose2d(0, 0, 0), hardwareMap);
        eh.localizer = drive.localizer;

        state = State.PRELOAD;

        intake = new Intake();
        outtake = new OutTakeMTI();

        while(opModeInInit()){
            outtake.update();
        }

        while(opModeIsActive()){


            drive.update(0, 0, 0);
            outtake.update();
            intake.update();
            cn.loop();

            ControlHub.telemetry.update();
        }

    }
}
