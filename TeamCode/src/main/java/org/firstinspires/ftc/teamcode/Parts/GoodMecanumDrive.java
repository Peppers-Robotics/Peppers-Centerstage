package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.xmlpull.v1.XmlPullParserException;

import java.util.ArrayList;

@Config
public class GoodMecanumDrive {
    public static MOTOR_PORTS leftFront = MOTOR_PORTS.M0, leftBack = MOTOR_PORTS.M1,
                    rightFront = MOTOR_PORTS.M2, rightBack = MOTOR_PORTS.M2;
    public boolean follow = false;
    public static PIDCoefficients transitionalCoeff = new PIDCoefficients(0, 0, 0),
                                    headingCoeff = new PIDCoefficients(0, 0, 0);
    public static PIDController transitionalPID = new PIDController(transitionalCoeff),
                                headingPID = new PIDController(headingCoeff);
    private Pose2d target;
    public Localizer localizer;
    public GoodMecanumDrive(){
        transitionalPID.setTargetPosition(0);
        headingPID.setTargetPosition(0);
    }

    public void setTargetPosition(Pose2d pos, HardwareMap hm){
        target = pos;
        if(localizer == null)
            localizer = new StandardTrackingWheelLocalizer(hm, new ArrayList<>(), new ArrayList<>());
    }
    public void setTargetPosition(Pose2d pos){
        target = pos;
    }

    public void update(double X, double Y, double rot) {
        localizer.update();

        if(follow){
            double xErr = localizer.getPoseEstimate().getX() - target.getX(),
                    yErr = localizer.getPoseEstimate().getY() - target.getY(),
                    rotErr = localizer.getPoseEstimate().getHeading() - target.getHeading();

            double module = Math.sqrt(xErr * xErr + yErr * yErr);
            double modulePow = transitionalPID.calculatePower(module);
            X = xErr * Math.cos(-localizer.getPoseEstimate().getHeading()) - yErr * Math.sin(-localizer.getPoseEstimate().getHeading());
            Y = xErr * Math.sin(-localizer.getPoseEstimate().getHeading()) + yErr * Math.cos(-localizer.getPoseEstimate().getHeading());
            rot = headingPID.calculatePower(rotErr);
            X *= -1;
        }

        double denominator = Math.max(1, Math.abs(X) + Math.abs(Y) + Math.abs(rot));

        double RB = X + Y - rot,
                LF = X - Y + rot,
                RF = X - Y - rot,
                LB = X + Y + rot;

        ExpansionHub.setMotorPower(leftFront, LF / denominator);
        ExpansionHub.setMotorPower(leftBack, LB / denominator);
        ExpansionHub.setMotorPower(rightBack, RB / denominator);
        ExpansionHub.setMotorPower(rightFront, RF / denominator);

    }
    public boolean isBuisy(){
        double xErr = localizer.getPoseEstimate().getX() - target.getX(),
                yErr = localizer.getPoseEstimate().getY() - target.getY(),
                rotErr = localizer.getPoseEstimate().getHeading() - target.getHeading();
        double module = Math.sqrt(xErr * xErr + yErr * yErr);

        return module < 0.3 && Math.abs(rotErr) < 5;
    }
}
