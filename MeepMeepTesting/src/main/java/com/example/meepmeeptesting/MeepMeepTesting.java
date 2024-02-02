package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18.95)
                .followTrajectorySequence(drive ->  drive.trajectorySequenceBuilder(new Pose2d(-35, -63, Math.toRadians(90)))
// to the backstage
                                .lineToLinearHeading(new Pose2d(-34.00, -29.00, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-49.00, -29.00, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-49.00, -15.00, Math.toRadians(90)))
                                .turn(Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(-52, -15, Math.toRadians(180.00)))
                                //spin away from wall
//                .lineToLinearHeading(new Pose2d(-49, -54, Math.toRadians(125.00)))
//                //crab walk past spike mark and drop off purple pixel
//                .lineToLinearHeading(new Pose2d(-49, -20, Math.toRadians(125.00)))
//                //align with pixel stack
//                .lineToLinearHeading(new Pose2d(-45.5, -17, Math.toRadians(180.00)))
//                .lineToLinearHeading(new Pose2d(-53.5 /* 54.5 */, -17, Math.toRadians(180.00)))


                                //release purple
                                .addDisplacementMarker(49,() -> {
                                })
                                .addDisplacementMarker(66,() -> {
                                })
                                .build()
                );

        RoadRunnerBotEntity myBlueBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18.95)
                .followTrajectorySequence(drive ->  drive.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(270)))
// to the backstage
                                .lineToLinearHeading(new Pose2d(-34.00, 26.00, Math.toRadians(270.00)))
                                .lineToLinearHeading(new Pose2d(-48.00, 26.00, Math.toRadians(270.00)))
                                .lineToLinearHeading(new Pose2d(-48.00, 20.00, Math.toRadians(270.00)))
                                .splineToLinearHeading(new Pose2d(-61, 6.5, Math.toRadians(180.00)), Math.toRadians(207.16))
                                .addDisplacementMarker(46, () -> {

                                })
                                .addDisplacementMarker(60, () -> {

                                })


                                //release purple
                                .addDisplacementMarker(51,() -> {
                                })
                                .addDisplacementMarker(62.25,() -> {
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBlueBot)
                .start();
    }
}