package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Avion;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.utils.PIDController;

import java.util.ArrayList;


@TeleOp(name = "old_pipers \uD83C\uDF36️")
@Config
public class MainOp extends LinearOpMode {

    public static Intake intake;
    public static OutTakeMTI outTake;
    public static SampleMecanumDriveCancelable drive;
    public static Controls c;
    public static Avion avion;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode(){
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>()));
        Controls ctr = new Controls(gamepad1, gamepad2);

//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        ControlHub.telemetry = telemetry;
        gamepad1.setLedColor(1.0, 1.0, 1.0, (int) 1e10);

        outTake = new OutTakeMTI();
        intake = new Intake();
        avion = new Avion();

        boolean boost = false;

        waitForStart();
        time.reset();
        while(opModeIsActive() && !isStopRequested()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();

            for(int i = 0; i < 4; i++){
                ControlHub.encoder[i].read = false;
                ExpansionHub.encoder[i].read = false;
            }



            boost = !gamepad1.cross;

//            ControlHub.teleMotorCurrents(telemetry);
//            ExpansionHub.teleMotorCurrents(telemetry);

            drive.setWeightedDrivePower( new Pose2d(
                    -gamepad1.left_stick_y * (boost ? 1.0 : 0.6),
                    -gamepad1.left_stick_x * (boost ? 1.0 : 0.6),
                    (gamepad1.left_trigger - gamepad1.right_trigger) * (boost ? 1.0 : 0.6)
            ));

            outTake.update();
            intake.update();
            avion.update();
            drive.update();
            e.update(true);

            intake.update_values();

//            outTake.runTelemetry();
//            intake.runTelemetry();

            ctr.loop();
            telemetry.addData("Hz", 1/time.seconds());
            e.teleAngle(telemetry);
            time.reset();

            telemetry.update();
        }
    }
}
