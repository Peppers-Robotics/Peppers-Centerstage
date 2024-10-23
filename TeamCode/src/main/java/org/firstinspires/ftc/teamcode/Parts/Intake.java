package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Exp;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.util.ResourceBundle;

/*
* MAP:
* 3 - INTAKE
* */
@Config
public class Intake implements Part {
    public static boolean Disabled = false;
    public enum STATES{
        IDLE,
        REVERSE,
        FORWARD
    }
    public static STATES STATE;
    public static double maxTrashHold = 1200;
    public static double ground = 235;
    private double usedCurrent = 0;
    public AutoServo servo;

    NanoClock clock;
    private double grippersHaveTime = 0;
    private boolean grippersHave = false;
    public static double[] stackPositions = {227, 215, 205, 197, 188};
//    public static double[] stackPositions = {125, 135, 145, 155, 165};
    public static double Up = 160;
    public static int lvl = 0;
    public static int TEST_POSITION = 0;

    public void setPixelStackPosition(int level){
        level--;
        if(level > 4) level = 4;
        if(level < 0) level = 0;
        lvl = level;
//        ground = stackPositions[level];
    }
    public int getPixelStackPosition(){
        return lvl;
    }

    public Intake(){
        STATE = STATES.IDLE;
        servo = new AutoServo(SERVO_PORTS.S5, 0,
                false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON);
        servo.setAngle(150);
        servo.update();
        clock = NanoClock.system();
        setPixelStackPosition(0);
    }
    private boolean close = false;
    private ElapsedTime manualTimer = new ElapsedTime();
    STATES prevState = STATES.IDLE;
    private void manualUpdate(){
        STATE = STATES.IDLE;
        if(Controls.Intake){
            STATE = STATES.FORWARD;
            prevState = STATES.FORWARD;
            manualTimer.reset();
        }
        if(Controls.RevIntake){
            STATE = STATES.REVERSE;
            prevState = STATES.REVERSE;
            manualTimer.reset();
        }

        close = manualTimer.seconds() >= 0.3;

        switch (STATE){
            case IDLE:
                ExpansionHub.setMotorPower(MOTOR_PORTS.M3, 0);
                servo.setAngle(Up);
                break;
            case FORWARD:
                ExpansionHub.setMotorPower(MOTOR_PORTS.M3, 1);
                servo.setAngle(stackPositions[lvl]);
                break;
            case REVERSE:
                ExpansionHub.setMotorPower(MOTOR_PORTS.M3, -1);
                servo.setAngle(Up);
                break;
        }


        servo.update();

    }
    public static boolean forceOut = false;
    public static double reversePower = -1;
    @Override
    public void update(){
        if(Disabled) return;
        if(Grippers.manualMode) manualUpdate();

        if(!OutTakeMTI.isFullOfPixels()) {
            grippersHaveTime = 0;
            grippersHave = false;
        } else if(grippersHaveTime == 0) {
            grippersHaveTime = clock.seconds();
        } else if(clock.seconds() - grippersHaveTime > 0.3) {
            grippersHave = true;
        }
        if((Controls.Intake && ExpansionHub.getCurrentFromMotor(MOTOR_PORTS.M3, CurrentUnit.MILLIAMPS) >= 6500) || Controls.RevIntake || (OutTakeMTI.left.hasAPixel() && OutTakeMTI.right.hasAPixel()) && Controls.Intake && grippersHave){
            STATE = STATES.REVERSE;
        } else if(Controls.Intake){
            STATE = STATES.FORWARD;
        }
        int power = -1;

        if(!Controls.RevIntake && !Controls.Intake) STATE = STATES.IDLE;

        switch (STATE){
            case IDLE:
                ExpansionHub.setMotorPower(MOTOR_PORTS.M3, 0);
                servo.setAngle(Up);
                break;
            case FORWARD:
                ExpansionHub.setMotorPower(MOTOR_PORTS.M3, 1);
                servo.setAngle(stackPositions[lvl]);
                break;
            case REVERSE:
                ExpansionHub.setMotorPower(MOTOR_PORTS.M3, reversePower);
                servo.setAngle(Up);
                break;
        }
        servo.update();

    }
    @Override
    public void update_values(){
        usedCurrent = ExpansionHub.getCurrentFromMotor(MOTOR_PORTS.M3, CurrentUnit.MILLIAMPS);
    }
    @Override
    public void runTelemetry(){}
}
