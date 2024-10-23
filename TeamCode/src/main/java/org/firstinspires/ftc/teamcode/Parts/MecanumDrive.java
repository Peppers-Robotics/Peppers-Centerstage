package org.firstinspires.ftc.teamcode.Parts;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

import java.util.ResourceBundle;

/*
* MAP:
* 0 - BR - R
* 1 - FR
* 2 - FL - R
* 3 - BL
*
* */
@Config
public class MecanumDrive {
    private static DcMotorEx FL, FR, BL, BR;
    private void setToMax(DcMotorEx m){
        MotorConfigurationType mct = m.getMotorType();
        mct.setAchieveableMaxRPMFraction(1);
        m.setMotorType(mct);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        m.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public MecanumDrive(HardwareMap hm){
        FL = hm.get(DcMotorEx.class, "eM2");
        FR = hm.get(DcMotorEx.class, "cM3");
        BL = hm.get(DcMotorEx.class, "eM0");
        BR = hm.get(DcMotorEx.class, "cM0");

        setToMax(FL);
        setToMax(FR);
        setToMax(BL);
        setToMax(BR);
    }

    public void update(double x, double y, double rot){
        double denom = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rot), 1);
        double flP = (y + x + rot) / denom;
        double frP = (y - x - rot) / denom;
        double blP = (y - x + rot) / denom;
        double brP = (y + x - rot) / denom;

        FL.setPower(flP);
        FR.setPower(frP);
        BL.setPower(blP);
        BR.setPower(brP);
    }

}
