package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.nio.charset.CharacterCodingException;

public class UltraSonicSensor {
    DigitalChannel echo, trigger;
    public UltraSonicSensor(DigitalChannel echoPort, DigitalChannel triggerPort){
        echo = echoPort;
        trigger = triggerPort;
        echo.setMode(DigitalChannel.Mode.INPUT);
        trigger.setMode(DigitalChannel.Mode.OUTPUT);
    }
    public double read() throws InterruptedException {
        trigger.setState(false);
        trigger.setState(true);
       /* long start = System.nanoTime();
        while(System.nanoTime() - start <= 2000){

        }
        trigger.setState(true);
        start = System.nanoTime();
        while (System.nanoTime() - start <= 10000){

        }

        trigger.setState(false);*/
        long time = System.nanoTime();
        while(!echo.getState() && System.nanoTime() - time <= 1000000){

        }
        double rawValue = System.nanoTime() - time;
        return rawValue * 1305118e-11/2.0;
    }
}
