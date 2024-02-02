package org.firstinspires.ftc.teamcode.Actual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous
public class PullOffStack extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);
        Pose2d start = new Pose2d(-36, 12, Math.toRadians(180));
        while (!isStarted()) {

        }
            drive.setPoseEstimate(start);
            TrajectorySequence toStack = drive.trajectorySequenceBuilder(start)
                    .lineTo(new Vector2d(-57.00, 11.25))
                    .lineTo(new Vector2d(-53.00,11.25))
                    .addDisplacementMarker(20,() -> {
                        drive.dropdown.setPosition(.410);
                    })
                    .addDisplacementMarker(28,() -> {
                        drive.intake.setPower(1);
                        drive.dropdown.setPosition(.300);
                    })
                    .lineTo(new Vector2d(-55.25, 11.25))
                    .lineTo(new Vector2d(-36,12))
                    .build();

            drive.followTrajectorySequence(toStack);

            if (isStopRequested()) return;
            stop();
    }
}
