package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;
import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.utils.AutoGamepad;

import java.io.File;
import java.io.IOException;

@TeleOp(name = "Create File")
public class CreateNewFile extends LinearOpMode {
    Telemetry tele;
    public static File file;
    public static void createFile(){
        if(!file.exists()){
            try {
                file.createNewFile();
            } catch (IOException e) {
                RobotLog.e("File not found");
            }
        }
    }
    public static void resetFile(){
        if(file.exists()){
            file.delete();
        }
        createFile();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        file = new File(Environment.getExternalStorageDirectory(), OutTakeMTI.cacheFileName);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tele = telemetry;

        AutoGamepad g1 = new AutoGamepad(gamepad1);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addLine("press cross to create a new file or square to reset the file");
            if(file.exists()){
                telemetry.addData("file usage", file.getUsableSpace());
            } else {
                telemetry.addData("file usage", 0);
            }
            if(g1.wasPressed.cross){
                telemetry.addLine("Creating File");
                createFile();
            }
            if (g1.wasPressed.square) {
                telemetry.addLine("Deleting File");
                file.delete();
            }

            telemetry.update();
            g1.update();
        }
    }
}
