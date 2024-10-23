package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.util.TypeConversion;

public class OnlyYawIMU extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public void setYawnEstimate(double estimate){

    }
    public class YawParameters{
        private byte[] initParam;

        public YawParameters(byte[] byteArray) {
            initParam = byteArray;
        }

        public byte[] getInitParam(){ return initParam; }
    }

    protected void write(REGISTERS reg, byte b){
        deviceClient.write8(reg.val, b);
    }
    protected byte read(REGISTERS reg){
        return deviceClient.read8(reg.val);
    }

    protected short readValue(REGISTERS regI){
        return TypeConversion.byteArrayToShort(deviceClient.read(regI.val, 2));
    }

    protected YawParameters readYawParameter(){
        write(REGISTERS.OPR_MODE, (byte) 0);
        byte[] saved = new byte[6];
        saved[0] = read(REGISTERS.GO_X_LSB);
        saved[1] = read(REGISTERS.GO_X_MSB);
        saved[2] = read(REGISTERS.GO_Y_LSB);
        saved[3] = read(REGISTERS.GO_Y_MSB);
        saved[4] = read(REGISTERS.GO_Z_LSB);
        saved[5] = read(REGISTERS.GO_Z_MSB);
        if(read(REGISTERS.CALIB_STAT) != 3) return null;
        return new YawParameters(saved);
    }

    protected void setYawParameter(YawParameters parameters){
        byte[] saved = parameters.getInitParam();

        write(REGISTERS.GO_X_LSB, saved[0]);
        write(REGISTERS.GO_X_MSB, saved[1]);
        write(REGISTERS.GO_Y_LSB, saved[2]);
        write(REGISTERS.GO_Y_MSB, saved[3]);
        write(REGISTERS.GO_Z_LSB, saved[4]);
        write(REGISTERS.GO_Z_MSB, saved[5]);

    }

    public OnlyYawIMU(I2cDeviceSynch device, boolean isOwned){
        super(device, isOwned);
        if(deviceClient instanceof LynxI2cDeviceSynch){
            ((LynxI2cDeviceSynch) deviceClient).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        }

        this.deviceClient.setI2cAddress(new I2cAddr(0x29));
        super.registerArmingStateCallback(false);

        this.deviceClient.engage();
    }



    @Override
    protected boolean doInitialize() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "BNO055 only Yaw IMU";
    }

    public enum REGISTERS{
        OPR_MODE(0x3d),
        PWR_MODE(0x3e),
        SYS_TRIGGER(0x3f),
        EULER_Heading_LSB(0x1a),
        EULER_Heading_MSB(0x1b),
        CHIP_ID(0),
        UNIT_SEL(0x3a),
        CALIB_STAT(0x35),
        GO_X_LSB(0x61),
        GO_X_MSB(0x62),
        GO_Y_LSB(0x63),
        GO_Y_MSB(0x64),
        GO_Z_LSB(0x65),
        GO_Z_MSB(0x66),
        ;

        REGISTERS(int hex){
            val = hex;
        }
        int val;
    }
}
