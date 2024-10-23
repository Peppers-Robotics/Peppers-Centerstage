package org.firstinspires.ftc.teamcode.utils;

public class LowPassFilter {
    private double prevValue, t;
    public LowPassFilter(double t){
        this.t = t;
    }
    public void setT(double t){
        this.t = t;
    }
    public double pass(double val){
        prevValue =  t * prevValue + (1 - t) * val;
        return prevValue;
    }
}
