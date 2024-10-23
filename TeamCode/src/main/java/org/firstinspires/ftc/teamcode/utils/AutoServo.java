package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class AutoServo {
    /*public static class EffectQueue{
        public enum Effect{
            MOVE;
            public double waitTime, moveTarget;
        }
        private final Queue<Effect> eff;
        public EffectQueue(){
            eff = new LinkedList<>();
        }
        public void addEffect(Effect x){
            eff.add(x);
        }
        private long time = 0;
        private double angle = 0;
        public double getAngleMovement(){
            ControlHub.telemetry.addData("elements", eff.size());
            if(eff.isEmpty()) return angle;
            if(time == 0) time = System.currentTimeMillis();
            ControlHub.telemetry.addData("waitTime", eff.peek().waitTime);

            if(System.currentTimeMillis() - time >= eff.peek().waitTime){
                //angle = eff.poll().moveTarget;
                time = System.currentTimeMillis();
                ControlHub.telemetry.addLine("poped");
            }

            return angle;
        }
    }*/
    public enum TYPE{
        DS,
        GOBILDA,
        MICRO_LEGO,
        AXON,
        UNKNOWN
    }
    private Hubs hub;
    private final SERVO_PORTS port;
    private final boolean isReversed;
    private double MAX_ANGLE;
    private double position;
    private final double initialPosition;
//    private EffectQueue queue;

    public AutoServo(SERVO_PORTS port, double initialPosition, boolean isReversed, Hubs hub, TYPE Type){
        this.port = port;
        this.initialPosition = initialPosition;
        this.isReversed = isReversed;
        this.hub = hub;
//        queue = new EffectQueue();

        switch (Type){
            case DS:
                MAX_ANGLE = 270;
                break;
            case GOBILDA:
                MAX_ANGLE = 300;
                break;
            case MICRO_LEGO:
                MAX_ANGLE = 180;
                break;
            case AXON:
                MAX_ANGLE = 355;
                break;
            case UNKNOWN:
                MAX_ANGLE = 360;
                break;
        }
        switch (hub){
            case CONTROL_HUB:
                ControlHub.setServoDirection(port, isReversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
                ControlHub.setServoPosition(port, position + initialPosition);
                break;
            case EXPANSION_HUB:
                ExpansionHub.setServoDirection(port, isReversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
                ExpansionHub.setServoPosition(port, position + initialPosition);
                break;
        }
    }


    public void update(){
//        position = queue.getAngleMovement();
        switch (hub){
            case CONTROL_HUB:
                ControlHub.setServoDirection(port, isReversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
                ControlHub.setServoPosition(port, position);
                break;
            case EXPANSION_HUB:
                ExpansionHub.setServoDirection(port, isReversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
                ExpansionHub.setServoPosition(port, position);
                break;
        }
    }

    public void setAngle(double angle){
//        setAngle(angle, 0);
        position = angle / MAX_ANGLE + initialPosition;
    }
    public void setPosition(double pos){
        position = pos + initialPosition;
        // NOT USED
    }

    public void setAngle(double angle, double delay){
        position = angle / MAX_ANGLE + initialPosition;
//        EffectQueue.Effect x = EffectQueue.Effect.MOVE;
//        x.waitTime = delay;
//        x.moveTarget = angle / MAX_ANGLE + initialPosition;
//        queue.addEffect(x);
    }

    public double getAngle(){return (position - initialPosition) * MAX_ANGLE; }
    public double getPosition(){ return position - initialPosition; }

}
