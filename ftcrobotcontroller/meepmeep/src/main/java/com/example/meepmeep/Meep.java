package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.swing.plaf.basic.BasicOptionPaneUI;

public class Meep {
    public static double x = 14, y = 62, rot = -90;
    public static Pose2d
            MiddlePurple = new Pose2d(22, -1, 0),
            MiddleYellow = new Pose2d(16.5, -29.5, Math.toRadians(286)),

    LeftPurple = new Pose2d(9.5, -9, Math.toRadians(-357)),
            LeftYellow = new Pose2d(12.5, -28, Math.toRadians(-76)),

    RightPurple = new Pose2d(15.5, 4, Math.toRadians(45)),
            RightYellow = new Pose2d(20, -29, Math.toRadians(-60))
                    ;

    public static Pose2d
            TrussToStack     = new Pose2d(3, 45, -Math.PI/2.f),
            Stack            = new Pose2d(22, 75, -Math.toRadians(110)),
            Stack2           = new Pose2d(34, 75, -Math.toRadians(110)),
            BackBoardToTruss = new Pose2d(3, 9, -Math.PI/2.f),
            Backdrop         = new Pose2d(14, -27.5, -Math.toRadians(80)),
            TrussToStack_s     = new Pose2d(3, 45, -Math.PI/2.f),
            BackBoardToTruss_s = new Pose2d(3, 9, -Math.PI/2.f);

    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, 5, 5, 12.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d())
                                .lineToLinearHeading(RightPurple)
//                                .splineToSplineHeading(new Pose2d(RightPurple.getX(), RightPurple.getY() - 5, Math.toRadians(-60)), Math.toRadians(-90))
                                .splineToSplineHeading(RightYellow, Math.toRadians(-90))
                                .build()


                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}