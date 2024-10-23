package org.firstinspires.ftc.teamcode.Parts;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.util.ResourceBundle;

@Config
public class Avion {
    public static int shoot_a = 0, keep = 60;
    public boolean shoot = false;
    public Avion(){
        shoot = true;
    }
    public void update(){
        ExpansionHub.setServoPosition(SERVO_PORTS.S2, shoot ? keep/180.f : shoot_a);
        if(Controls.Avion && !Controls.AvionAck) {
            Controls.AvionAck = true;
            shoot = !shoot;
        }
    }
}
