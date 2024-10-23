package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Avion;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;

@TeleOp(name = ".pipers \uD83C\uDF36")
@Config
public class MainOpMTI extends LinearOpMode {
    long lastTime = 0;
    public static boolean MEASURE_CHASSIS_POWER_COSUMPTION = false;
    SampleMecanumDriveCancelable mecanumDrive;
    public enum GamePadAxiesCompensation{
        G1LX(0.01),
        G1LY(0.01),
        G1RX(0.01),
        G1RY(0.01),
        G1LT(0),
        G1RT(0),

        G2LX(0.01),
        G2LY(0.01),
        G2RX(0.01),
        G2RY(0.01),
        G2LT(0),
        G2RT(0),
        ;
        GamePadAxiesCompensation(double treshHold){
            TH = treshHold;
        }
        public double TH;
    }


    public static double DeadZoneFilter(double rawValue, GamePadAxiesCompensation AXIES){
        return Math.abs(rawValue) < AXIES.TH ? 0 : rawValue;
    }
    Avion avion;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        ExpansionHub.IMU_FREQ = 100; // TODO: REMOVE WHEN GATA ODO
        mecanumDrive = new SampleMecanumDriveCancelable(hardwareMap);
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, mecanumDrive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);
        ControlHub.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        drive = new MecanumDrive(hardwareMap);

        OutTakeMTI out = new OutTakeMTI();
        Intake intake = new Intake();
        avion = new Avion();
        for(int i = 0; i < 3; i++){
            ControlHub.motor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        boolean boost = true;
        boolean DownIntake = false, UpIntake = false, stackMode = false;
        int StackPos = 5, savedStackPos = 0;
        while(opModeInInit()){
            out.update();
        }
        lastTime = System.currentTimeMillis();

        while (opModeIsActive()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();
            if(Controls.ResetTourret) {
                ExpansionHub.resetIMU();
                Controls.ResetTourretAck = true;
            }
            if(stackMode) {
                if (gamepad2.right_stick_y <= -0.7) {
                    if (!DownIntake) {
                        StackPos++;
                        if (StackPos > 5) StackPos = 5;
                        intake.setPixelStackPosition(StackPos);
                    }
                    DownIntake = true;
                } else DownIntake = false;
                if (gamepad2.right_stick_y >= 0.7) {
                    if (!UpIntake) {
                        StackPos--;
                        if (StackPos < 1) StackPos = 1;
                        intake.setPixelStackPosition(StackPos);
                    }
                    UpIntake = true;
                } else UpIntake = false;
            }
            if(Controls.StackMode){
                Controls.StackModeAck = true;
                stackMode = !stackMode;
                if(!stackMode){
                    intake.setPixelStackPosition(1);
                    StackPos = 5;
                } else {
                    intake.setPixelStackPosition(StackPos);
                }
            }

            e.update(false);
            if(Controls.IntakeLvlUp){
                ControlHub.telemetry.addLine("sjhjsdh");
            }

            boost = !gamepad1.a;

//            mecanumDrive.setWeightedDrivePower(  new Pose2d(
//                -gamepad1.left_stick_y * (boost ? 1.0 : 0.3),
//                -gamepad1.left_stick_x * (boost ? 1.0 : 0.3),
//                (gamepad1.left_trigger - gamepad1.right_trigger) * (boost ? 1.0 : 0.3)
//            ));

            drive.update(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_trigger - gamepad1.left_trigger);

            ControlHub.telemetry.addData("freq", 1000.f/(System.currentTimeMillis() - lastTime));
            lastTime = System.currentTimeMillis();

            out.update();
            intake.update();
            intake.update_values();
            mecanumDrive.update();
            e.teleAngle(ControlHub.telemetry);

            cn.loop();
            ControlHub.telemetry.addData("robot tilt", ExpansionHub.tiltAngle);
            ControlHub.telemetry.addData("stack pos", StackPos);

            if(MEASURE_CHASSIS_POWER_COSUMPTION){
                double power = 0;
                for(int i = 0; i < 3; i++) {
                    power += ControlHub.motor[i].getCurrent(CurrentUnit.MILLIAMPS);
                    ControlHub.telemetry.addData("motor" + i, ControlHub.motor[i].getCurrent(CurrentUnit.MILLIAMPS));
                }
            }
            avion.update();
            ControlHub.telemetry.update();
        }

    }
}
