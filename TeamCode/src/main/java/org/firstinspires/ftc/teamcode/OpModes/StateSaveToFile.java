package org.firstinspires.ftc.teamcode.OpModes;

import android.content.Context;
import android.os.Environment;

import androidx.core.content.ContextCompat;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;
import com.sun.tools.doclint.Env;

import org.java_websocket.extensions.IExtension;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.StringBufferInputStream;
import java.util.Objects;

@TeleOp(name = "StateSaveToFile")
@Config
public class StateSaveToFile extends LinearOpMode {

    public static final String cachedFileName = "cachedPositions.op";
    public static boolean writeToFile = true;
    public static String data = "extended";
    public static boolean Operate = false;
    public File savedDataFile, savedDataDirectory;
    public void WriteToFile(){
        FileOutputStream out;
        try {
            out = new FileOutputStream(savedDataFile);
            out.write(data.getBytes());
            out.close();
        } catch (IOException e) {
            RobotLog.e("file \"" + cachedFileName + "\" appears to not exist, try running createFile() before");
            stop();
        }
    }
    public String readData(){
        FileInputStream in;
        try {
            in = new FileInputStream(savedDataFile);
            InputStreamReader reader = new InputStreamReader(in);
            BufferedReader buffer = new BufferedReader(reader);

            StringBuffer stringBuffer = new StringBuffer();
            String text = null;

            while((text = buffer.readLine()) != null){
                stringBuffer.append(text);
            }
            buffer.close();

            return stringBuffer.toString();
        } catch (IOException e) {
            RobotLog.e("file \"" + cachedFileName + "\" appears to not exist, try running createFile() before");
            stop();
        }
        return null;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if(Objects.equals(Environment.getExternalStorageState(), Environment.MEDIA_MOUNTED))
            RobotLog.v("robot states will be periodically saved on the sdCard");
        else RobotLog.v("can't find any external storage, make sure that the sdCard is mounted on the ControlHub");

        waitForStart();
        savedDataFile = new File(Environment.getExternalStorageDirectory(), cachedFileName);


        if(!savedDataFile.exists()){
            try {
                savedDataFile.createNewFile();
            } catch (IOException e) {
                RobotLog.e("can't create new file \"" + cachedFileName + "\"");
                stop();
            }
        }

        waitForStart();

        while (opModeIsActive()){
            if(Operate){
                if(writeToFile){
                    WriteToFile();
                } else {
                    telemetry.addLine(readData());
                }
            }

            telemetry.update();

        }
    }
}
