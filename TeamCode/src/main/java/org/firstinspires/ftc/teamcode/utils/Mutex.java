package org.firstinspires.ftc.teamcode.utils;

public class Mutex {
    private boolean isLocked = false;
    public boolean kill = false;
    public void lock(){
        while(isLocked);
        isLocked = true;
    }
    public void unlock(){
        isLocked = false;
    }
    public void kill(){kill = true;}
}
