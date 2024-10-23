package org.firstinspires.ftc.teamcode.Auto.AutoUtils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Parts.GoodMecanumDrive;

import java.nio.file.Path;
import java.sql.Array;
import java.util.Vector;

public class Paths {
    private GoodMecanumDrive drive;
    public Vector<Pose2d> points;
    public Paths(GoodMecanumDrive d){
        drive = d;
    }
    public void addPoint(Pose2d point){
        points.add(point);
    }
    private int counter = 0;

    public void update(){
        if(!drive.isBuisy()){
            drive.setTargetPosition(points.elementAt(counter));
            counter ++;
        }
    }
}
