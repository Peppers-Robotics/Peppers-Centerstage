package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.Parts.MotionProfile;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@Config
public class ElevatorArm implements Part {

    private AutoServo virtual1, turret, rotation;
    private MotionProfile armProfile = new MotionProfile(4000, 2000);
    public static double currentArmAngle = 0, defaultTouretDegrees = 209, imuResetedAngle = 0;
    public ElevatorArm() {
        virtual1 = new AutoServo(SERVO_PORTS.S0, 75.f/355.f, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
        virtual1.setAngle(OutTakeMTI.armAngleIntake);
        virtual1.update();
        virtual1.update();
        virtual1.update();
        virtual1.update();

        rotation = new AutoServo(SERVO_PORTS.S4,  164.f/355.f,false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
        rotation.setAngle(0);
        rotation.update();

        turret = new AutoServo(SERVO_PORTS.S3, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
        turret.setAngle(defaultTouretDegrees);
        turret.update();
        rotationIndex = 2;
    }
    public static double[] rotationAngles = {-98, -65, 0, 60, 90};
    public int rotationIndex;
    public enum Direction {
        LEFT,
        RIGHT
    }
    public void rotatePixels(Direction dir){
        switch (dir){
            case LEFT:
                rotationIndex --;
                if(rotationIndex < 0) rotationIndex = 0;
                break;
            case RIGHT:
                rotationIndex ++;
                if(rotationIndex > 4) rotationIndex = 4;
                break;
        }
    }
    public void setArmAngle(double angle){
        virtual1.setAngle(angle);
    }
    private long timePivot = 0, timeElapsed = 0;
    public void setPivotAngle(double angle, long time){
        timePivot = time;
        timeElapsed = System.currentTimeMillis();
    }
    public void setPivotAngle(double angle){
        setPivotAngle(angle, 0);
    }

    public void setOrientation(double angle){
        while(angle > 180) angle -= 360;
        while(angle < -180) angle += 360;

        if(Math.abs(angle) > 50) angle = 50 * Math.signum(angle);

        turret.setAngle(angle + defaultTouretDegrees);
    }
    public void setPixelRotation(double angle){
        rotation.setAngle(angle);
    }
    public double getPixelRotation(){ return rotation.getAngle(); }
    public double getArmAngle(){
        return virtual1.getAngle();
    }
    public double getLiveArmAngle(){
        return armProfile.getPosition();
    }

    public boolean reachedStationary(){
        return true;
    }
    @Override
    public void update(){
        rotation.update();
        virtual1.update();
        turret.update();

    }
    @Override
    public void update_values(){
    }
    @Override
    public void runTelemetry(){
        ControlHub.telemetry.addData("arm angle", getArmAngle());
        ControlHub.telemetry.addData("turret angle", turret.getAngle());
    }

}