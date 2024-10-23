package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

public class MecanumDriveVectorial {
    private Pose2d currentPose;
    private Vector2d heading;
    private Localizer localizer;

    public MecanumDriveVectorial(){
        ExpansionHub.setMotorDirection(MOTOR_PORTS.M0, DcMotorSimple.Direction.REVERSE);
        ExpansionHub.setMotorDirection(MOTOR_PORTS.M1, DcMotorSimple.Direction.REVERSE);
    }

    public void update(){

    }
}
