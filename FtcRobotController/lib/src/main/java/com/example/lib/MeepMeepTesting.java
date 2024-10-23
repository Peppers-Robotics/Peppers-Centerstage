package com.example.lib;

import static java.lang.Math.min;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double
                stack_x = 50, stack_y = 18.5, stack_h = Math.toRadians(-90),
                park_x = 49, park_y = -92, park_h = Math.toRadians(-90),
                middlepurple_x = 37, middlepurple_y = 14.5, middlepurple_h = Math.toRadians(-90),
                leftpurple_x = 10.5, leftpurple_y = 7, leftpurple_h = Math.toRadians(0),
                rightpurple_x = 16, rightpurple_y = -3, rightpurple_h = Math.toRadians(300),
                middleyellow_x = 32, middleyellow_y = -87, middleyellow_h = Math.toRadians(-110);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 30, 3, Math.toRadians(180), 13.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d())//new Pose2d(leftyellow_x-12, leftyellow_y - 1 - Y_OFFSET, Math.toRadians(-55)))
                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(leftpurple_x, 0, leftpurple_h))
                                .splineToSplineHeading(new Pose2d(stack_x - 5, 4, stack_h), -stack_h / 2)
                                .splineToConstantHeading(new Vector2d(stack_x, stack_y), -stack_h)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}