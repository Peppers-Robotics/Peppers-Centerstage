package org.firstinspires.ftc.teamcode.internals;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Mat;

public class Encoder {
    public boolean read = false;
    private double read_pos = 0;
    public enum Direction{
        FORWARD,
        REVERSE
    }
    private Direction direction;
    private DcMotorEx motor;
    public Encoder(DcMotorEx m){
        motor = m; direction = Direction.FORWARD;
    }

    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDirection(Direction dir){
        direction = dir;
    }

    public double getPosition(){
        if(!read) {
            double normalizePos = motor.getCurrentPosition();
            read_pos = direction == Direction.FORWARD ? normalizePos : -normalizePos;
            read = true;
        }
        return read_pos;
    }
    public double getVelocity(){
        return Math.abs(motor.getVelocity());
    }
}
