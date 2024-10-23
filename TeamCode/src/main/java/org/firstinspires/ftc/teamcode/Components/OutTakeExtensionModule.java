package org.firstinspires.ftc.teamcode.Components;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Parts.AsymetricMotionProfile;
import org.firstinspires.ftc.teamcode.Parts.MotionProfile;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

@Config
public class OutTakeExtensionModule {
    private AutoServo servo1, servo2;
    enum State{
        EXTENDING,
        RETRACTING
    }

    public State state = State.RETRACTING;

    public static double retractS1 = 50,
                         extendS1 = 195;

    public AsymetricMotionProfile profile = new AsymetricMotionProfile(300000, 2000000, 1000);

    public OutTakeExtensionModule(){
        servo1 = new AutoServo(SERVO_PORTS.S0, 0, false, Hubs.EXPANSION_HUB, AutoServo.TYPE.AXON);
        servo2 = new AutoServo(SERVO_PORTS.S5, 0, false, Hubs.EXPANSION_HUB, AutoServo.TYPE.AXON);
        retract();
    }

    public void update(){
        if(state == State.RETRACTING) {
            servo1.setAngle(retractS1);
            servo2.setAngle(retractS1);
            profile.setInstant(retractS1);
            servo1.update();
            servo2.update();
        } else {
            profile.update();
            servo1.setAngle(profile.getPosition());
            servo2.setAngle(profile.getPosition());
            servo1.update();
            servo2.update();
        }
    }

    public void extend(){
        profile.startMotion(retractS1, extendS1);
        state = State.EXTENDING;
        update();
    }
    public void retract(){
        state = State.RETRACTING;
        update();
    }
    public double getExtensionAngle(){
        double value = ControlHub.ExtensionEncoder.getVoltage();
        return value / 3.3;
    }
    public boolean reachedStationary(){
        return getExtensionAngle() == extendS1 || getExtensionAngle() == retractS1;
    }
    public boolean isExtended(){
        return profile.motionEnded();
    }
    public boolean almostExtended() {
        return abs(profile.getPosition() - extendS1) < 10;
    }
    public boolean isRetracted(){
        return getExtensionAngle() >= 0.8;
    }
}
