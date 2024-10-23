package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Part;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.firstinspires.ftc.teamcode.utils.BetterColorRangeSensor;
import org.firstinspires.ftc.teamcode.utils.LowPassFilter;

@Config
public class Grippers implements Part {
    public static boolean manualMode = false;
    public enum State{
        OPEN,
        CLOSE
    }
    public State state;
    public BetterColorRangeSensor sensor;
    private AutoServo servo;
    private final ElapsedTime time = new ElapsedTime();

    public Grippers(AutoServo servo, DigitalChannel sensor){
        this.servo = servo;

        sensor.setMode(DigitalChannel.Mode.INPUT);
        state = State.OPEN;
        manualMode = false;
        update();
        update_values();
    }
    public Grippers(AutoServo servo, BetterColorRangeSensor sensor, int treshHold){
        sensor.setThresHold(treshHold);
        this.servo = servo;
        this.sensor = sensor;
        state = State.OPEN;
        manualMode = false;
        update();
        update_values();
    }
    public Grippers(AutoServo servo, BetterColorRangeSensor sensor, int treshHold, double open, double close){
        opend = open;
        closed = close;
        sensor.setThresHold(treshHold);
        this.servo = servo;
        this.sensor = sensor;
        state = State.OPEN;
        manualMode = false;
        servo.setAngle(opend);
        servo.update();
        update();
        update_values();
    }
    public void open() {
        state = State.OPEN;
    }
    public void close(){
        state = State.CLOSE;
    }
    public double closed = 0, opend = 0;
    private void manualUpdate(){
        switch (state){
            case OPEN:
                servo.setAngle(opend);
                break;
            case CLOSE:
                servo.setAngle(closed);
                break;
        }
        servo.update();
    }

    @Override
    public void update(){
        if(manualMode) manualUpdate();
        switch (state){
            case OPEN:
                servo.setAngle(opend);
                break;
            case CLOSE:
                servo.setAngle(closed);
                break;
        }
        servo.update();
    }
    public void update_values(boolean updateBasedOnSensors){
        if(manualMode) return;
        if(updateBasedOnSensors){
            if(sensor.LogicProximityStatus()) state = State.CLOSE;
            else state = State.OPEN;
        }
    }
    public boolean hasAPixel(){
        return sensor.LogicProximityStatus();
    }

    @Override
    public void update_values(){
        if(manualMode) return;
        if(OutTake.state == OutTake.State.WAITING_FOR_PIXELS){
            if(sensor.LogicProximityStatus()){
                state = State.CLOSE;
            } else {
                state = State.OPEN;
            }
        }

        update();

        servo.update();
    }

    public void drop(){
        state = State.OPEN;
        update();
    }

    @Override
    public void runTelemetry(){ }

    public void runTelemetry(String s){

        ControlHub.telemetry.addLine();
        ControlHub.telemetry.addData("name", s);
        ControlHub.telemetry.addData("State", state.toString());
        ControlHub.telemetry.addData("readings", sensor.getProximityDistance());

    }

}