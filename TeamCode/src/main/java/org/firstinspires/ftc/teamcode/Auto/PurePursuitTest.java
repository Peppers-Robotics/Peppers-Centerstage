package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.utils.HoldPosition;

import java.util.Vector;

@Autonomous(name = "PurePursuit test")
public class PurePursuitTest extends LinearOpMode {
    PurePursuit path;
    HoldPosition holdPosition;
    SampleMecanumDriveCancelable drive;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        holdPosition = new HoldPosition(telemetry, drive.getLocalizer());
        Vector<Pose2d> list = new Vector<>();
        list.add(new Pose2d(0, 0));
        list.add(new Pose2d(3 * 24, Math.PI/2));
        list.add(new Pose2d(3 * 24, 3 * 24, Math.PI));
        list.add(new Pose2d(0, 3 * 24, Math.PI));
//        list.add(new Pose2d(0 ,0));
        path = new PurePursuit(drive.getLocalizer(), list, 5);

        waitForStart();

        while (opModeIsActive()){
            Pose2d pos = path.getNextHeadingPoint();
            holdPosition.holdPos(pos.getX(), pos.getY(), pos.getHeading());
            double[] l = holdPosition.update();
            drive.setWeightedDrivePower(new Pose2d(
                l[0],
                -l[1],
                l[2]
            ));
            drive.update();
            telemetry.addData("next x", pos.getX());
            telemetry.addData("next y", pos.getY());
            telemetry.addData("next heading", pos.getHeading());

            telemetry.update();

        }
    }
}
