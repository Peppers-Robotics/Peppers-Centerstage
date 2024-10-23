package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.Parts.MotionProfile;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.firstinspires.ftc.teamcode.utils.LowPassFilter;
import org.opencv.core.Mat;

@Config
public class OutTakeExtension implements Part {
    private DistanceSensor sensor;
    private AutoServo servo;
    public static double length, angle;
    private Telemetry telemetry = ControlHub.telemetry;
    public static boolean preactive = false;
    public static boolean active = false;
    public static double armLenghtInMM = -200;
    private MotionProfile extensionProfile;

    public static double Start = -210;
    public static double End = -240;
    public static double Vel = 3000;
    public static double Accel = 4000;
    public boolean MOTION_PROFILED = false;

    public OutTakeExtension(DistanceSensor sensor, AutoServo servo){

        this.sensor = sensor;
        this.servo = servo;
        extensionProfile = new MotionProfile(Vel, Accel);
        extensionProfile.startMotion(1, 0);

    }

    public void preactivate() {
        preactive = true;
    }

    public void activate(){
        preactive = true;
        active = true;
    }
    public void deactivate(){
        preactive = false;
        active = false;
    }
    public void setImuAngle(double angle){
        if(angle < 0) angle += 2 * Math.PI;
        this.angle = angle;
    }
    private static final double t_min = 66.5, a = 29.8, b = 65, c = 134.387;
    private double getServoAngleByLenght(double l){
        if(Double.isNaN(l)) return 0;

        double T = l / 120.0;
        l += Start * (1 - T) + End * T;

        l += t_min;

        double theta = - (c*c - a*a - l*l - b*b) / (2*b*Math.sqrt(a*a + l*l));
        if(Math.abs(theta) > 1) theta = 1 * Math.signum(theta);
        theta = Math.acos(theta);
        theta = Math.toDegrees(theta);

        double sAngle = l / Math.sqrt(a * a + l * l);
        if(Math.abs(sAngle) > 1) sAngle = 1 * Math.signum(sAngle);
        sAngle = Math.asin(sAngle);
        if(sAngle < 0) sAngle += Math.PI / 2;

        return 220 - theta - Math.toDegrees(sAngle);
    }
    private double lastA = 0;
    private boolean last_active = false;
    @Override
    public void update(){
        if(!MOTION_PROFILED) {
            double a = Math.min(getServoAngleByLenght(length), 118);
            if (Double.isNaN(a)) a = lastA;
            if (!active) a = 0;

            servo.setAngle(a);
            servo.update();
            lastA = a;

            if (Controls.DownElevator) sensor.resetDeviceConfigurationForOpMode();
        } else {
            if(last_active != active)  {

                if(active) extensionProfile.startMotion(0, 118);
                else extensionProfile.startMotion(118, 0);

                last_active = active;
            }

            extensionProfile.update();
            servo.setAngle(extensionProfile.getPosition());
            servo.update();
        }
    }

    public double getLivePosition () {
        return extensionProfile.getPosition();
    }

    public boolean reachedStationary() {
        return extensionProfile.motionEnded();
    }

    @Override
    public void update_values(){
        if(!MOTION_PROFILED) {
            angle = ExpansionHub.ImuYawAngle;
            angle = Math.toRadians(angle);
            if (angle < 0) angle += 2 * Math.PI;

            length = ExpansionHub.extension_length - armLenghtInMM / Math.cos(angle);
            if (preactive && !active) length -= 60;
        }
    }

    @Override
    public void runTelemetry(){
        telemetry.addLine("\n------- EXTENSION --------");
        telemetry.addData("recorded length", length);
        telemetry.addData("raw servo angle", getServoAngleByLenght(length));
        telemetry.addData("angle: ", angle);
        telemetry.addData("armAngleCompensation: ",  - armLenghtInMM / Math.cos(angle));
        telemetry.addData("extensionProfile: ",  extensionProfile.getPosition());}
}
