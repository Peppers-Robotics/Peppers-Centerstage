package org.firstinspires.ftc.teamcode.Parts;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtensionModule;
import org.firstinspires.ftc.teamcode.OpModes.MainOpMTI;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class OutTakeMTI {
    public static boolean DISABLE = false;
    public enum State{
        WAIT_FOR_PIXELS,
        EXTENDING,
        EXTENDED,
        RETRACTING,
        RETRACTED,
        PLACING_PIXELS,
        PLACE_PURPLE_1,
        PLACE_PURPLE_2,
        RESET_OUTTAKE,
        HANG;
        public static double level = 3;
    }

    public static State state = null;
    public static final int MAX_EXTEND = 1360;
    public static final int STEP = (MAX_EXTEND) / 11;
    public int joystickElevatorOffset = 0;
    public static int MAX_LVL = 11;
    public static double safeToExtendOuttake = STEP * 2.5;
    public static double armAnglePlaceingBackboard = 85, armAnglePlacingPurple = 0,
                        armAngleIntake = 27, armAngleRetracting = 15, intakeRotation = -3, tourretOffset = -2;
    public static Elevator elevator = null;
    public static ElevatorArm arm = null;
    public static OutTakeExtensionModule extension = null;
    public static Grippers left = null, right = null;
    private static double pixelsAngle = 0;
    public static double slowmo = 1;
    public static final String cacheFileName = "outTakeStates.pep";
    public static File cacheFile;
//    public static void writeStateToFile(State toWrite){
//        FileOutputStream out;
//        try {
//            out = new FileOutputStream(cacheFile);
//
//            out.write((toWrite.toString() + "\n").getBytes(), 0, toWrite.toString().length()+1);
//
//            out.close();
//        } catch (IOException e) {
//            RobotLog.ee("file exception", "cache file not found, try running \"Create File\" teleOp");
//        }
//    }
//    public static State readStateFromFile(){
//        FileInputStream in;
//        try {
//            in = new FileInputStream(cacheFile);
//            InputStreamReader reader = new InputStreamReader(in);
//            BufferedReader buffer = new BufferedReader(reader);
//
//            String tmp = buffer.readLine();
//            if(tmp == null) return State.WAIT_FOR_PIXELS;
//            tmp = tmp.replaceAll("[^a-zA-Z0-9_]", "");
//
//
//            return State.valueOf(tmp);
//        } catch (IOException e) {
//            RobotLog.ee("file exception", "cache file not found, try running \"Create File\" teleOp");
//        }
//        return State.WAIT_FOR_PIXELS;
//
//    }
    public static int treshHoldR = 570, treshHoldL = 520;

    public OutTakeMTI(){
        cacheFile = new File(Environment.getExternalStorageDirectory(), cacheFileName);
        state = State.WAIT_FOR_PIXELS;
        State.level = 5;
        extension = new OutTakeExtensionModule();

        elevator = new Elevator();
        arm = new ElevatorArm();
        right = new Grippers(new AutoServo(SERVO_PORTS.S1, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON),
                    ControlHub.right, 600, 10.f, 175.f);
        left = new Grippers(new AutoServo(SERVO_PORTS.S2, 0, false, Hubs.CONTROL_HUB, AutoServo.TYPE.AXON),
                    ControlHub.left, 600, 327.f, 130.f);
        if(DISABLE) return;
        align = false;
        arm.setArmAngle(armAngleIntake);
        arm.setPixelRotation(0);
        elevator.setInstantPosition(-60);

        extension.update();
        arm.update();
        elevator.update();
        elevator.update_values();
        left.update();
        right.update();
        driverUpdated = false;
        update();
    }
    public void changeStateTo(State s){
        state = s;
//        new Thread(() -> {
//            writeStateToFile(s);
//        }).start();
    }
    public static boolean align = false, waitForTimer = false;
    public boolean setToPurplePlace = false;
    ElapsedTime armAndExtendTime = new ElapsedTime(), startRetraction = new ElapsedTime();
    public void setToPurplePlacing(){
        State.level = 1;
        setToPurplePlace = true;
        changeStateTo(State.EXTENDING);
    }
    public static boolean hasAPixel(){
        return left.hasAPixel() || right.hasAPixel();
    }
    public static boolean reverse = false;
    public static boolean isInAutonomous = false;
    private void controls(){
        joystickElevatorOffset += (int) (-MainOpMTI.DeadZoneFilter(Controls.gamepad2.right_stick_y, MainOpMTI.GamePadAxiesCompensation.G2RY) * 7);
        if(state == State.PLACING_PIXELS && joystickElevatorOffset != 0){
            updateElevator();
        }
        if(Controls.Mirror){
            Controls.Mirror = false;
            arm.rotationIndex = 4 - arm.rotationIndex;
        }
        if(Controls.Hang) {

            changeStateTo(State.HANG);
            Controls.HangAck = true;
        }
        if(Controls.DropLeft) {
            Controls.DropLeftAck = true;
            left.drop();
        }
        if(Controls.DropRight) {
            Controls.DropRightAck = true;
            right.drop();
        }
        if(Controls.rotateRight) {
            Controls.rotateRightAck = true;
            arm.rotatePixels(ElevatorArm.Direction.LEFT);
        }
        if(Controls.rotateLeft){
            Controls.rotateLeftAck = true;
            arm.rotatePixels(ElevatorArm.Direction.RIGHT);
        }

        if(Controls.ExtendElevator) {
            setToPurplePlace = false;
            Controls.ExtendElevatorAck = true;
            driverUpdated = true;
            time1.reset();
//            if(left.state == Grippers.State.CLOSE) left.close();
//            if(right.state == Grippers.State.CLOSE) right.close();
            changeStateTo(State.EXTENDING);
        }
        if(Controls.RetractElevator) {
            left.open();
            right.open();
            Controls.RetractElevatorAck = true;
            setToPurplePlace = false;
            if (state == State.WAIT_FOR_PIXELS) {
                changeStateTo(State.RESET_OUTTAKE);
                startRetraction.reset();
            } else {
                changeStateTo(State.RETRACTING);
                waitForTimer = false;
                startRetraction.reset();
            }
        }
        if(Controls.ElevatorUp){
            Controls.ElevatorUpAck = true;
            State.level++;
            if(State.level > 9) State.level = 9;
//            if(state != State.WAIT_FOR_PIXELS) elevator.setTargetPosition(State.level * STEP + joystickElevatorOffset);
            if(state == State.PLACING_PIXELS) elevator.setLevel((int) State.level);
        }
        else if(Controls.ElevatorDown){
            Controls.ElevatorDownAck = true;
            State.level--;
            if(State.level < 0) State.level = 0;
//            if(state != State.WAIT_FOR_PIXELS) elevator.setTargetPosition(State.level * STEP + joystickElevatorOffset);
            if(state == State.PLACING_PIXELS) elevator.setLevel((int) State.level);
        }
    }
    public void updateElevator(){
        if(State.level > MAX_EXTEND) State.level = MAX_EXTEND;

//        while(State.level < 7 && Elevator.PixelLayer[(int) State.level] + joystickElevatorOffset > Elevator.PixelLayer[(int) State.level + 1]){
//            State.level ++;
//            joystickElevatorOffset -= Elevator.PixelLayer[(int)State.level] - Elevator.PixelLayer[(int)State.level - 1] ;
//        }
//        while(State.level > 1 && Elevator.getPositionByLevel((int) State.level) + joystickElevatorOffset < Elevator.getPositionByLevel((int) State.level)){
//            State.level --;
//            joystickElevatorOffset = Elevator.getPositionByLevel((int) State.level + 1) - Elevator.getPositionByLevel((int) State.level) - joystickElevatorOffset;
//        }
        elevator.setTargetPosition(Elevator.PixelLayer[(int) State.level] + joystickElevatorOffset);
    }
    public static boolean driverUpdated = true;
    private ElapsedTime time1 = new ElapsedTime();
    public void setToNormalPlacingFromPurplePixelPlacing(){
        arm.setPixelRotation(ElevatorArm.rotationAngles[arm.rotationIndex]);
        waitForTimer = false;
        setToPurplePlace = false;
        align = true;
    }
    public static double extendingAngle = -100;

    public void update(){
        if(DISABLE) return;
        right.sensor.setThresHold(treshHoldR);
        left.sensor.setThresHold(treshHoldL);
        controls();
        switch (state){
            case WAIT_FOR_PIXELS:
                if(isInAutonomous) {
                    right.update_values(true);
                    left.update_values(true);
                } else {
                    if(Intake.STATE == Intake.STATES.FORWARD){
                        right.open();
                        left.open();
                    } else {
                        right.close();
                        left.close();
                    }
                }
                arm.setPixelRotation(intakeRotation);
                arm.setArmAngle(armAngleIntake);
                arm.setOrientation(tourretOffset);
                extension.retract();
                joystickElevatorOffset = 0;
                // sensors responsive
                break;
            case EXTENDING:
                if(elevator.targetPosition != Math.max(safeToExtendOuttake + 10, Elevator.getPositionByLevel((int) State.level)))
                    elevator.setTargetPosition(Math.max(safeToExtendOuttake + 10, Elevator.getPositionByLevel((int) State.level)));
                if(time1.seconds() >= 0.25){
                    if(arm.rotationIndex <= 2)
                        arm.setPixelRotation(extendingAngle);
                    else arm.setPixelRotation(-extendingAngle);
                    arm.setArmAngle(armAnglePlacingPurple); // to be parallel with the ground
                    changeStateTo(State.EXTENDED);
                    waitForTimer = false;
                    time1.reset();
                }
                break;
            case EXTENDED:
                if(elevator.getLivePosition() >= safeToExtendOuttake && !waitForTimer){
                    extension.extend();
                    armAndExtendTime.reset();
                    waitForTimer = true;
                    time1.reset();
                }
                if(waitForTimer && extension.almostExtended()){
                    waitForTimer = false;
                    arm.setPixelRotation(pixelsAngle);
//                    elevator.setTargetPosition(STEP * State.level + joystickElevatorOffset);
                    if(setToPurplePlace) elevator.setTargetPosition(0);
                    else elevator.setLevel((int) State.level);
                    changeStateTo(State.PLACING_PIXELS);
                    time1.reset();
                }
                break;
            case RETRACTING:
                joystickElevatorOffset = 0;
                if(elevator.targetPosition < safeToExtendOuttake) elevator.setTargetPosition(safeToExtendOuttake);
                driverUpdated = false;
                if(!waitForTimer) {
                    align = false;
                    arm.setOrientation(0);
                    arm.setArmAngle(armAnglePlacingPurple);
                    if(arm.rotationIndex <= 2)
                        arm.setPixelRotation(extendingAngle);
                    else arm.setPixelRotation(-extendingAngle);
                    if(time1.seconds() >= 0.15 * slowmo) {
                        waitForTimer = true;
                        armAndExtendTime.reset();
                        extension.retract();
                    }
                } else if(/*extension.isRetracted() &&*/ armAndExtendTime.seconds() >= 0.5 * slowmo && !retractBoolean){
                    time1.reset();
                    arm.setPixelRotation(intakeRotation);
                    arm.update();
                    retractBoolean = true;
                }
                if(time1.seconds() >= 0.1 * slowmo && retractBoolean){
                    arm.setArmAngle(armAngleRetracting);
                    arm.setPixelRotation(intakeRotation);
                    arm.update();
                    elevator.setTargetPosition(-60);
                    retractBoolean = false;
                    changeStateTo(State.RETRACTED);
                }
                break;
            case RETRACTED:
                if(elevator.reatchedTargetPosition()){
                    changeStateTo(State.WAIT_FOR_PIXELS);
                }
                break;
            case PLACING_PIXELS:
                if(!setToPurplePlace) {
                    arm.setPixelRotation(ElevatorArm.rotationAngles[arm.rotationIndex]);
                    arm.setArmAngle(armAnglePlaceingBackboard);
                    if(time1.seconds() >= 0.1) {
                        align = true;
                    }
                } else {
                    arm.setArmAngle(armAnglePlacingPurple);
                    arm.setPixelRotation(ElevatorArm.rotationAngles[arm.rotationIndex]);
                    arm.setArmAngle(0);
                    align = false;
                }
                if(left.state == Grippers.State.OPEN && right.state == Grippers.State.OPEN && driverUpdated){
                    if(!dropTime){
                        droping.reset();
                        dropTime = true;
                    } else if(droping.seconds() >= timeToDrop * slowmo) {
                        dropTime = false;
                        changeStateTo(State.RETRACTING);
                        waitForTimer = false;
                        startRetraction.reset();
                    }
                }
                break;
            case HANG:
                State.level = 7;
                changeStateTo(State.EXTENDING);
                break;
            case RESET_OUTTAKE:
                elevator.setTargetPosition(safeToExtendOuttake);
                if(startRetraction.seconds() >= 0.3) {
                    elevator.setTargetPosition(-60);
                }
                if(elevator.reatchedTargetPosition() && elevator.targetPosition == -60){
                    changeStateTo(State.WAIT_FOR_PIXELS);
                }
                break;
            case PLACE_PURPLE_1:
                elevator.setTargetPosition(STEP * 6);
//                elevator.setLevel();
                if(arm.rotationIndex <= 2)
                    arm.setPixelRotation(extendingAngle);
                else arm.setPixelRotation(-extendingAngle);
                arm.setArmAngle(armAnglePlacingPurple); // to be parralel with ground
                changeStateTo(State.PLACE_PURPLE_2);
                waitForTimer = false;
                time1.reset();
                break;
            case PLACE_PURPLE_2:
                if(elevator.getLivePosition() >= safeToExtendOuttake + 100 && time1.seconds() >= 0.6 && !waitForTimer){
                    elevator.setTargetPosition(safeToExtendOuttake);
                    extension.extend();
                    time1.reset();
                    waitForTimer = true;
                } else if(time1.seconds() >= 0.5){
                    arm.setPixelRotation(0);
                    elevator.setTargetPosition(-60);
                }
                break;
        }
        if(align)
            arm.setOrientation(ExpansionHub.ImuYawAngle);
        elevator.update();
        arm.update();

        left.update();
        right.update();
        elevator.update_values();
        arm.update_values();
        extension.update();


        ControlHub.telemetry.addData("state", state.toString());
        right.runTelemetry("right");
        left.runTelemetry("left");
        elevator.runTelemetry();
        if(right.state == Grippers.State.CLOSE && rightLast == Grippers.State.OPEN) gotRightPixel = true;
        if(left.state == Grippers.State.CLOSE && leftLast == Grippers.State.OPEN) gotLeftPixel = true;
    }
    public static double timeToDrop = 0.3;
    private Grippers.State rightLast = Grippers.State.OPEN, leftLast = Grippers.State.OPEN;
    private boolean gotLeftPixel = false, gotRightPixel = false, retractBoolean = false;
    public boolean gotAPixel(){
        if(gotRightPixel){
            gotRightPixel = false;
            return true;
        }
        if(gotLeftPixel){
            gotLeftPixel = false;
            return true;
        }
        return false;
    }
    private ElapsedTime droping = new ElapsedTime();
    private boolean dropTime = false;
    public static boolean isFullOfPixels(){
        return left.hasAPixel() && right.hasAPixel();
    }
}