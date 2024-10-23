package org.firstinspires.ftc.teamcode.Parts;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.opencv.core.Mat;

public class MotionProfile {
    private double maxVelocity, acceleration;
    private ElapsedTime time = new ElapsedTime();
    private double accelerationTime, deccelarationTime, constantTime,
                    currentPosition, initialPosition, targetPosition, mvUsed,
                    velocity, sig;
    private double t0 = 0, t1 = 0, t2 = 0;
    public MotionProfile(double maxVelocity, double acceleration){
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
    }
    public void startMotion(double initialPos, double targetPos){
        if(initialPos == targetPos) return;
        double dist = Math.abs(targetPos - initialPos);
        double mv = dist / Math.sqrt(dist / acceleration);
        sig = Math.signum(targetPos - initialPos);
        initialPosition = initialPos;
        targetPosition = targetPos;
        currentPosition = initialPosition;

        accelerationTime = deccelarationTime = maxVelocity / acceleration;

        if(dist <= accelerationTime * maxVelocity){
            constantTime = 0;
            accelerationTime = deccelarationTime = mv / acceleration;
            mvUsed = mv;
        } else {
            constantTime = dist / maxVelocity - accelerationTime;
            mvUsed = maxVelocity;
        }

        t0 = accelerationTime;
        t1 = accelerationTime + constantTime;
        t2 = t1 + deccelarationTime;


        velocity = 0;
        time.reset();
    }
    private double a(double t){
        if(t <= t0) return acceleration;
        if(t <= t1) return 0;
        if(t <= t2) return -acceleration;
        return 0;
    }
    private double v(double t){
        if(t <= t0) return t * acceleration;
        if(t <= t1) return maxVelocity;
        if(t <= t2) return maxVelocity - acceleration * (t - t1);
        return 0;
    }
    private double p(double t){
        if(t <= t0) return acceleration / 2 * t * t;
        if(t <= t1) return acceleration / 2 * t0 * t0 + mvUsed * (t - t0);
        if(t <= t2) return acceleration / 2 * t0 * t0 + mvUsed * (t - t0) - acceleration / 2 * (t - t1) * (t - t1);
        return 0;
    }
    public void update(){
//        currentPosition += sig * p(time.seconds());
        if(time.seconds() < t2)
            currentPosition = initialPosition + sig * p(time.seconds());
        else currentPosition = targetPosition;
//        ControlHub.telemetry.addData("mp position", p(time.seconds()));
//        ControlHub.telemetry.addData("time", time.seconds());
//        ControlHub.telemetry.addData("t0", t0);
//        ControlHub.telemetry.addData("t1", t1);
//        ControlHub.telemetry.addData("t2", t2);
//        ControlHub.telemetry.addData("acceleration", acceleration);
//        ControlHub.telemetry.addData("currentPos", currentPosition);
    }
    public double getPosition(){
        return currentPosition;
    }
    public boolean motionEnded(){
        return time.seconds() >= t2;
    }
}