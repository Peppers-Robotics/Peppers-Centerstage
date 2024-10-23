package org.firstinspires.ftc.teamcode.internals;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.BetterColorRangeSensor;
import org.firstinspires.ftc.teamcode.utils.Mutex;

import java.security.cert.Extension;
import java.util.List;

public class ControlHub {
    public static boolean disableDevice = false;
    public static Telemetry telemetry;

    public static DcMotorEx[] motor = new DcMotorEx[4];
    public static Encoder[] encoder = new Encoder[4];
    public static Servo[] servo = new Servo[6];
    private static final double[] servo_cache = new double[6];
    private static final double[] motor_cache = new double[4];
    private static final double[] motor_target_cache = new double[4];

    public static double voltage;
    public static double compensation = 12;
    public static IMU imu;
    public static BetterColorRangeSensor left, right;
    private static void setMotorsToMax(){
        MotorConfigurationType mct = motor[0].getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor[0].setMotorType(mct);

        mct = motor[1].getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor[1].setMotorType(mct);

        mct = motor[1].getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor[2].setMotorType(mct);

        mct = motor[3].getMotorType().clone();
        mct.setAchieveableMaxRPMFraction(1.0);
        motor[3].setMotorType(mct);

    }
    public static LynxModule ControlHubModule;
    public static AnalogInput ExtensionEncoder;

    public ControlHub(HardwareMap hm){
        motor[0] = hm.get(DcMotorEx.class, "cM0");
        motor[1] = hm.get(DcMotorEx.class, "cM1");
        motor[2] = hm.get(DcMotorEx.class, "cM2");
        motor[3] = hm.get(DcMotorEx.class, "cM3");
        ExtensionEncoder = hm.get(AnalogInput.class, "cA0");

        left = hm.get(BetterColorRangeSensor.class, "leftSensor");
        right = hm.get(BetterColorRangeSensor.class, "rightSensor");

        left.addLowPassFilter(1);
        right.addLowPassFilter(1);
        setMotorsToMax();

        for(int i = 0; i < 4; i++){
            motor[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_cache[i] = 69;
            motor_target_cache[i] = 69;
        }

        for(int i = 0; i < 6; i++){
            servo[i] = hm.get(Servo.class, "cS" + i );
            servo_cache[i] = 69;
        }
        for(int i = 0; i < 4; i++){
            encoder[i] = new Encoder(motor[i]);
        }

        voltage = hm.voltageSensor.iterator().next().getVoltage();

        ControlHubModule = hm.getAll(LynxModule.class).get(0);

        ControlHubModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        compensation = voltage;

    }
    public static double getEncoderPosition(ENCODER_PORTS encoder_port){
        if(disableDevice) return 0;
        switch(encoder_port){
            case E0:
                return encoder[0].getPosition();
            case E1:
                return encoder[1].getPosition();
            case E2:
                return encoder[2].getPosition();
            case E3:
                return encoder[3].getPosition();
        }
        return 0;
    }
    public static double getMotorVelocity(ENCODER_PORTS encoder){
        if(disableDevice) return 0;
        switch (encoder){
            case E0:
                return motor[0].getVelocity();
            case E1:
                return motor[1].getVelocity();
            case E2:
                return motor[2].getVelocity();
            case E3:
                return motor[3].getVelocity();
        }
        return 0;
    }
    public static void setServoPosition(SERVO_PORTS port, double position){
        if(disableDevice) return;
        switch (port) {
            case S0:
                if(servo_cache[0] != position) {
                    servo[0].setPosition(position);
                    servo_cache[0] = position;
                }
                break;
            case S1:
                if(servo_cache[1] != position) {
                    servo[1].setPosition(position);
                    servo_cache[1] = position;
                }
                break;
            case S2:
                if(servo_cache[2] != position) {
                    servo[2].setPosition(position);
                    servo_cache[2] = position;
                }
                break;
            case S3:
                if(servo_cache[3] != position) {
                    servo[3].setPosition(position);
                    servo_cache[3] = position;
                }
                break;
            case S4:
                if(servo_cache[4] != position) {
                    servo[4].setPosition(position);
                    servo_cache[4] = position;
                }
                break;
            case S5:
                if(servo_cache[5] != position) {
                    servo[5].setPosition(position);
                    servo_cache[5] = position;
                }
                break;
        }
    }
    public static double getCurrentFromMotor(MOTOR_PORTS port, CurrentUnit unit){
        if(disableDevice) return 0;
        switch (port){
            case M0:
                return motor[0].getCurrent(unit);
            case M1:
                return motor[1].getCurrent(unit);
            case M2:
                return motor[2].getCurrent(unit);
            case M3:
                return motor[3].getCurrent(unit);
        }
        return 0;
    }
    public static void setServoDirection(SERVO_PORTS port, Servo.Direction dir) {
        if(disableDevice) return;
        switch (port) {
            case S0:
                servo[0].setDirection(dir);
                break;
            case S1:
                servo[1].setDirection(dir);
                break;
            case S2:
                servo[2].setDirection(dir);
                break;
            case S3:
                servo[3].setDirection(dir);
                break;
            case S4:
                servo[4].setDirection(dir);
                break;
            case S5:
                servo[5].setDirection(dir);
                break;
        }
    }
    public static void setMotorDirection(MOTOR_PORTS port, DcMotorSimple.Direction dir){
        if(disableDevice) return;
        switch (port){
            case M0:
                motor[0].setDirection(dir);
                break;
            case M1:
                motor[1].setDirection(dir);
                break;
            case M2:
                motor[2].setDirection(dir);
                break;
            case M3:
                motor[3].setDirection(dir);
                break;
        }
    }
    public static void setMotorTargetPosition(MOTOR_PORTS port, int position){
        if(disableDevice) return;
        switch (port){
            case M0:
                if(motor_target_cache[0] != position){
                    motor[0].setTargetPosition(position);
                    motor_target_cache[0] = position;
                }
                break;
            case M1:
                if(motor_target_cache[1] != position){
                    motor[1].setTargetPosition(position);
                    motor_target_cache[1] = position;
                }
                break;
            case M2:
                if(motor_target_cache[2] != position){
                    motor[2].setTargetPosition(position);
                    motor_target_cache[2] = position;
                }
                break;
            case M3:
                if(motor_target_cache[3] != position){
                    motor[3].setTargetPosition(position);
                    motor_target_cache[3] = position;
                }
                break;
        }
    }
    public static void setMotorPower(MOTOR_PORTS port, double power){
        if(disableDevice) return;
        switch (port){
            case M0:
                if(motor_cache[0] != power){
                    motor[0].setPower(power * compensation / voltage);
                    motor_cache[0] = power;
                }
                break;
            case M1:
                if(motor_cache[1] != power){
                    motor[1].setPower(power * compensation / voltage);
                    motor_cache[1] = power;
                }
                break;
            case M2:
                if(motor_cache[2] != power){
                    motor[2].setPower(power * compensation / voltage);
                    motor_cache[2] = power;
                }
                break;
            case M3:
                if(motor_cache[3] != power){
                    motor[3].setPower(power * compensation / voltage);
                    motor_cache[3] = power;
                }
                break;
        }
    }
    public static void setEncoderDirection(ENCODER_PORTS port, Encoder.Direction dir){
        if(disableDevice) return;
        switch (port){
            case E0:
                encoder[0].setDirection(dir);
                break;
            case E1:
                encoder[1].setDirection(dir);
                break;
            case E2:
                encoder[2].setDirection(dir);
                break;
            case E3:
                encoder[3].setDirection(dir);
                break;
        }
    }
    public static void resetEncoder(ENCODER_PORTS port){
        if(disableDevice) return;
        switch (port){
            case E0:
                encoder[0].reset();
                break;
            case E1:
                encoder[1].reset();
                break;
            case E2:
                encoder[2].reset();
                break;
            case E3:
                encoder[3].reset();
                break;
        }
    }

    public static void teleMotorCurrents(Telemetry telemetry) {
        if(disableDevice) return;

        telemetry.addData("EM0:", getCurrentFromMotor(MOTOR_PORTS.M0, CurrentUnit.AMPS));
        telemetry.addData("EM1:", getCurrentFromMotor(MOTOR_PORTS.M1, CurrentUnit.AMPS));
        telemetry.addData("EM2:", getCurrentFromMotor(MOTOR_PORTS.M2, CurrentUnit.AMPS));
        telemetry.addData("EM3:", getCurrentFromMotor(MOTOR_PORTS.M3, CurrentUnit.AMPS));
        telemetry.addData("Expansion Power:", getCurrentFromMotor(MOTOR_PORTS.M0, CurrentUnit.AMPS) + getCurrentFromMotor(MOTOR_PORTS.M1, CurrentUnit.AMPS) + getCurrentFromMotor(MOTOR_PORTS.M2, CurrentUnit.AMPS) + getCurrentFromMotor(MOTOR_PORTS.M3, CurrentUnit.AMPS));

    }


}
