package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;
import java.nio.ByteOrder;

@I2cDeviceType
@DeviceProperties(xmlTag = "REVColorSensorv3", name = "REV Color Sensor v3 - proximity")
public class BetterColorRangeSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public void setReadWindow(I2cDeviceSynch.ReadMode mode){
        this.deviceClient.setReadWindow(new I2cDeviceSynch.ReadWindow(
                0x00,
                0x1E - 0x07 + 1,
                mode
        ));
    }

    protected short readProximity(REGISTERS r){
        return TypeConversion.byteArrayToShort(deviceClient.read(r.reg, 2), ByteOrder.LITTLE_ENDIAN);
    }
    protected void writeToDevice(final REGISTERS r, short val){
        deviceClient.write(r.reg, TypeConversion.shortToByteArray(val, ByteOrder.LITTLE_ENDIAN));
    }

    protected byte readByte(REGISTERS r){
        return deviceClient.read8(r.reg);
    }

    /**
     * This is a very expensive operation DO NOT USE IT OUTSIDE OF INIT!
     *
     *
     * default value = 0
     *
     * @param val the threshold value for close signal to be triggered
     * */

    private int thVal = 0;
    public void setThresHold(int val){
        thVal = val;
//        byte[] send = {(byte) (val & 0xFF), (byte) (val & 0x0700)};
//        deviceClient.write8(REGISTERS.PS_THRES_UP_0.reg, send[0]);
//        deviceClient.write8(REGISTERS.PS_THRES_UP_1.reg, send[1]);
    }
    public short getTrashHold(){
        return TypeConversion.byteArrayToShort(deviceClient.read(REGISTERS.PS_THRES_LOW_0.reg, 2), ByteOrder.LITTLE_ENDIAN);
    }
    private double precent = 1;
    public void addLowPassFilter(double p){
        precent = p;
    }


    private short last = 10, lastReadedValue = 10;
    private double lastTime = 0;

    public short getProximityDistance(){
        if(System.currentTimeMillis() - lastTime >= 50){
            last = TypeConversion.byteArrayToShort(deviceClient.read(REGISTERS.PS_DATA_0.reg, 2), ByteOrder.LITTLE_ENDIAN);
//            last >>= 3;
            last = (short) (last * precent + lastReadedValue * (1 - precent));
            lastTime = System.currentTimeMillis();
            lastReadedValue = last;
        }
        return last;
    }

    public BetterColorRangeSensor(I2cDeviceSynch device, boolean isOwned) throws Exception {
        super(device, isOwned);

        if(device == null){
            throw new Exception("cum ba??");
        }
        if(deviceClient instanceof LynxI2cDeviceSynch){
            ((LynxI2cDeviceSynch) deviceClient).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        }

        this.setReadWindow(I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x52));
//


        this.registerArmingStateCallback(true);

        this.deviceClient.engage();
    }

    public byte getDeviceMainCtrl(){
        return deviceClient.read8(REGISTERS.MAIN_STATUS.reg);
    }

    public void enable(){
        byte notUsedRead = readByte(REGISTERS.MAIN_CTRL);


        //enable PS
        deviceClient.write8(REGISTERS.MAIN_CTRL.reg, 0x1);

        notUsedRead = readByte(REGISTERS.MAIN_CTRL);
        RobotLog.vv(getDeviceName(), "enabled proximity 0x%02x", notUsedRead);
    }
    public enum Unit{
        MM,
        CM,
        INCH

    }
    private double f1(double x){
        return 170000 / (x * x) + 400;
    }
    private double f2(double x){
        return 30 * Math.exp(-1/8.f * (x - 10) * (x - 10)) + f1(10);
    }

    public double getDistance(Unit unit){
        double read = getProximityDistance();
        double mm = 0;
        if(read < 10) mm = f2(read);
        else mm = f1(read);

        switch (unit){
            case MM:
                return mm;
            case CM:
                return mm * 10;
            case INCH:
                return mm * 25.4;
        }
        return mm;
    }

    @Override
    protected boolean doInitialize() {
        deviceClient.write8(0x1, 0b00110111);
        deviceClient.write8(0x2, 32);

        deviceClient.write8(0x3, 0b00011100);
        deviceClient.write8(0x19, 0x11);
        enable();

        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    public byte getManufacturerRawID(){
        return readByte(REGISTERS.PART_ID);
    }

    @Override
    public String getDeviceName() {
        return "REV ColorSensor v3 - proximity";
    }
    public boolean LogicProximityStatus(){
        return thVal <= getProximityDistance();
    }
    /*
    *
    * 0010000
    * 0000100
    * */

    public enum REGISTERS{
        MAIN_CTRL(0x0),
        PS_LED(0x1),
        PS_PULSES(0x2),
        PS_MEAS_RATE(0x3),
        LS_MEAS_RATE(0x4),
        LS_Gain(0x5),
        PART_ID(0x6),
        MAIN_STATUS(0x7),
        PS_DATA_0(0x8), // LSB
        PS_DATA_1(0x9), // MSB

        PS_THRES_LOW_0(0x1D),
        PS_THRES_LOW_1(0x1E),
        PS_THRES_UP_0(0x1B),
        PS_THRES_UP_1(0x1C);

        REGISTERS(int hex){
           reg = hex;
        }
        int reg = 0x0;
    }

}
