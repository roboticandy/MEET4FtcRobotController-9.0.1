package org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actual.CustomDriveFunctions;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutoPixelPickupTest extends LinearOpMode {
    private Drive drive;
    private double speed;
    private double time;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-62.5,6, Math.toRadians(180)));
        while(!isStarted()){
            telemetry.addData("Parallel Count: ",drive.rightRear.getCurrentPosition());
            telemetry.update();
        }
                TrajectorySequence intakePull = drive.trajectorySequenceBuilder(new Pose2d(-62.5,6, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-58.5,6,Math.toRadians(180)),
                        drive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence intakeIt = drive.trajectorySequenceBuilder(intakePull.end())
                .lineToLinearHeading(new Pose2d(-64,6,Math.toRadians(180)),
                        drive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-63.25,6,Math.toRadians(180)),
                        drive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

                drive.dropdown.setPosition(.385);
                straightTime(drive,.25 , 2);
                straight(drive,-.25,-.125);
                drive.dropdown.setPosition(.37);
                sleep(250);
                drive.intake.setPower(1);
                straight(drive,-.5,-3);
//                drive.followTrajectorySequence(intakePull);
                drive.dropdown.setPosition(.6);
                sleep(100);
                straight(drive,.5,3);
//                drive.followTrajectorySequence(intakeIt);
                drive.dropdown.setPosition(.355);
                sleep(250);
                drive.intake.setPower(1);
                straight(drive,-.5,-3);
//                drive.followTrajectorySequence(intakePull);
                drive.dropdown.setPosition(.6);
                sleep(100);
                straight(drive,.5,3);
                straight(drive,-.5,-12);
//                drive.followTrajectorySequence(intakeIt);


//        TrajectorySequence forwardThree = drive.trajectorySequenceBuilder(new Pose2d(-58.5,6, Math.toRadians(180)))
//                .lineToLinearHeading(new Pose2d(-62.5,6,Math.toRadians(180)),
//                        drive.getPowerConstraint(6, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
//                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//        TrajectorySequence intakeIt = drive.trajectorySequenceBuilder(forwardThree.end())
//                .lineToLinearHeading(new Pose2d(-64.5,6,Math.toRadians(180)),
//                        drive.getPowerConstraint(6, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
//                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .lineToLinearHeading(new Pose2d(-58.5,6,Math.toRadians(180)),
//                        drive.getPowerConstraint(6, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
//                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//        drive.dropdown.setPosition(.37);
//        drive.followTrajectorySequence(forwardThree);
//        drive.intake.setPower(1);
//        sleep(500);
//        drive.dropdown.setPosition(.7);
//        sleep(250);
//        drive.followTrajectorySequence(intakeIt);
//        drive.intake.setPower(0);
//        drive.dropdown.setPosition(.36);
//        drive.followTrajectorySequence(forwardThree);
//        drive.intake.setPower(1);
//        sleep(500);
//        drive.dropdown.setPosition(.7);
//        sleep(250);
//        drive.followTrajectorySequence(intakeIt);

        stop();

    }
    public void straight(Drive drive, double speed, double distance){
        double currentParallelCount = drive.rightRear.getCurrentPosition();
        double countToAdd = distance * 336.7;
        if(distance < 0) {
            while (drive.rightRear.getCurrentPosition() < currentParallelCount - countToAdd && opModeIsActive()) {
                drive.leftFront.setPower(speed);
                drive.leftRear.setPower(speed);
                drive.rightFront.setPower(speed);
                drive.rightRear.setPower(speed);
            }
        } else{
            while (drive.rightRear.getCurrentPosition() > currentParallelCount - countToAdd && opModeIsActive()) {
                drive.leftFront.setPower(speed);
                drive.leftRear.setPower(speed);
                drive.rightFront.setPower(speed);
                drive.rightRear.setPower(speed);
            }
        }
        drive.leftFront.setPower(0);
        drive.leftRear.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightRear.setPower(0);
        sleep(100);
    }
    public void straightTime(Drive drive, double speed, double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
            while (timer.time() < seconds && opModeIsActive()) {
                drive.leftFront.setPower(speed);
                drive.leftRear.setPower(speed);
                drive.rightFront.setPower(speed);
                drive.rightRear.setPower(speed);
            }
        drive.leftFront.setPower(0);
        drive.leftRear.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightRear.setPower(0);
        sleep(100);
    }
}
