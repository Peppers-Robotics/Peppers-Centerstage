package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

public class RumbleEffects {

    public static Gamepad.RumbleEffect fullLoad, pixelLost, LeftPixel, RightPixel, LeftLost, RightLost, ENDGAME, FINISHED;

    public RumbleEffects(){
        Gamepad.RumbleEffect.Builder
                fullLoad_b = new Gamepad.RumbleEffect.Builder(),
                pixelLost_b = new Gamepad.RumbleEffect.Builder(),
                LeftPixel_b = new Gamepad.RumbleEffect.Builder(),
                RightPixel_b = new Gamepad.RumbleEffect.Builder(),
                LeftLost_b = new Gamepad.RumbleEffect.Builder(),
                RightLost_b = new Gamepad.RumbleEffect.Builder(),
                ENDGAME_b = new Gamepad.RumbleEffect.Builder(),
                FINISHED_b = new Gamepad.RumbleEffect.Builder();

        LeftPixel_b.addStep(1, 0, 500);
        RightPixel_b.addStep(0, 1, 500);

        fullLoad_b.addStep(0.4, 0.4, 500);
        fullLoad_b.addStep(1, 1, 1500);

        pixelLost_b.addStep(1, 1, 1000);
        LeftLost_b.addStep(1, 1, 1000);
        LeftLost_b.addStep(0.5, 0, 250);
        LeftLost_b.addStep(0.5, 0, 250);

        RightLost_b.addStep(1, 1, 1000);
        RightLost_b.addStep(0, 0.5, 250);
        RightLost_b.addStep(0, 0.5, 250);

        ENDGAME_b.addStep(1, 1, 800);

        FINISHED_b.addStep(1, 1, 1500);
        FINISHED_b.addStep(1, 1, 1500);
        FINISHED_b.addStep(1, 1, 1500);

        fullLoad = fullLoad_b.build();
        pixelLost = pixelLost_b.build();
        LeftLost = LeftLost_b.build();
        LeftPixel = LeftPixel_b.build();
        RightPixel = RightPixel_b.build();
        RightLost = RightLost_b.build();
        ENDGAME = ENDGAME_b.build();
        FINISHED = FINISHED_b.build();


    }

    public static Gamepad.LedEffect ledEffect;

    public static void runLEDEffect(){
        new Thread(() -> {

        });
    }
}
