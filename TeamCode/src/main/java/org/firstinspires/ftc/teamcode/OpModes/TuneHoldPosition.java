package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.Pair;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.utils.HoldPosition;

import java.util.ArrayList;
import java.util.Vector;

@TeleOp(name = "tuneHoldPos")
@Config
public class TuneHoldPosition extends LinearOpMode {
    public static double x = 0, y = 0, heading = 0;
    public static double X = 0, Y = 0, H = 0;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDriveCancelable md = new SampleMecanumDriveCancelable(hardwareMap);
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap, new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>()));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HoldPosition hp = new HoldPosition(telemetry, md.getLocalizer());
        hp.holdPos(0, 0, 0);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();
            double[] l = hp.update();
            md.setWeightedDrivePower(new Pose2d(
                    l[0],
                    -l[1],
                    l[2]
            ));

            hp.holdPos(x, y, heading);
            md.update();
            telemetry.update();
        }

    }
}
