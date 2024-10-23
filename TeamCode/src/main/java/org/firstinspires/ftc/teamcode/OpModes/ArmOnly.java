package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Parts.MotionProfile;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.util.ArrayList;

@Config
@TeleOp(name = "armOnly")
public class ArmOnly extends LinearOpMode {
    public static double armAngle = 0, tourretAngle = 355/2, pivot = 0, init = 0, init2 = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap, new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>()));
        ControlHub.telemetry = telemetry;


        AutoServo virtual1 = new AutoServo(SERVO_PORTS.S0, 0, false, Hubs.EXPANSION_HUB, AutoServo.TYPE.AXON);
        AutoServo virtual2 = new AutoServo(SERVO_PORTS.S2, 0, true, Hubs.EXPANSION_HUB, AutoServo.TYPE.AXON);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            virtual1.setAngle(armAngle + init);
            virtual2.setAngle(armAngle + init2);

            virtual1.update();
            virtual2.update();
        }
    }
}
