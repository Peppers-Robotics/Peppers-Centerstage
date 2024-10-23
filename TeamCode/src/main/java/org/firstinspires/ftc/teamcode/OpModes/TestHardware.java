package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtension;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.util.ArrayList;

@Config
@TeleOp(name = "testHardware")
public class TestHardware extends LinearOpMode {
    OutTake outTake;
    @Override
    public void runOpMode() throws InterruptedException{
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub expansionHub = new ExpansionHub(hardwareMap, new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>()));
        telemetry = FtcDashboard.getInstance().getTelemetry();

        outTake = new OutTake(hardwareMap);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){

            outTake.update();
            outTake.update_values();

            telemetry.update();
        }

    }
}
