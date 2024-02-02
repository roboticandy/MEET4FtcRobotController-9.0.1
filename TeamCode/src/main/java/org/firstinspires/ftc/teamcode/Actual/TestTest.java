package org.firstinspires.ftc.teamcode.Actual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.trajectorysequence.TrajectorySequence;
@Autonomous
@Disabled
public class TestTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);
        Pose2d start = new Pose2d(-35, 63, Math.toRadians(270));
        while (!isStarted()) {

        }
//        TrajectorySequence BlueCenterProp = drive.trajectorySequenceBuilder(start)
////                .splineTo(new Vector2d(10.97, 34.51), Math.toRadians(270))
//                .setReversed(true)
//                .lineTo(new Vector2d(-37.71, 15))
//                .build();
        drive.setPoseEstimate(start);
        TrajectorySequence ExtraPixel = drive.trajectorySequenceBuilder(start)

//                .back(6)
//                .turn(Math.toRadians(90))
//                .setReversed(true)
                .lineTo(new Vector2d(-37.71, 15))
                .lineTo(new Vector2d(-54.00, 11.25))
                .lineTo(new Vector2d(-53.00,11.25))
                .lineTo(new Vector2d(-55.25, 11.25))
                .addDisplacementMarker(50,() -> {
                    drive.dropdown.setPosition(.410);
                })
                .addDisplacementMarker(60,() -> {
                    drive.intake.setPower(1);
                    drive.dropdown.setPosition(.300);
                })
                .build();
        drive.followTrajectorySequence(ExtraPixel);
        if (isStopRequested()) return;
        stop();
    }
}
