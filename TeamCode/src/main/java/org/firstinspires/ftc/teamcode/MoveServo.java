package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class MoveServo extends LinearOpMode {

    Servo s;
    boolean zero = true, prev = false;
    public void runOpMode() throws InterruptedException {
        s = hardwareMap.get(Servo.class,"servo");
        waitForStart();
        while(opModeIsActive()){
        if((gamepad1.a && zero)&& !prev )
        {
            zero = false;
            s.setPosition(1);}

        else if(gamepad1.a &&!zero&&!prev)
        {
            zero = true;
            s.setPosition(0);}
        prev = gamepad1.a;}
    }
}
