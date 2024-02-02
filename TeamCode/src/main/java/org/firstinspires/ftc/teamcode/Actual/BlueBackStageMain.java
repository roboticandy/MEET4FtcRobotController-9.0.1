package org.firstinspires.ftc.teamcode.Actual;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
@Autonomous
public class BlueBackStageMain extends LinearOpMode {
    public static int propPos = 1;

    @Override
    public void runOpMode() {
        OpenCvWebcam webcam;
        Drive drive = new Drive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        BlueCircleDetector propDetector = new BlueCircleDetector();
        webcam.setPipeline(propDetector);
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        int slowerVelocity = 30;
        Pose2d startPose = new Pose2d(12, 63, Math.toRadians(270));
        while (!isStarted()) {

            {
                if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) { // CENTER
                    telemetry.addLine("Detected Prop! Position:CENTER");
                    telemetry.update();


                } else if (propDetector.centerOfProp.x >= 200) { //center
                    telemetry.addLine("Detected Prop! POSITION:RIGHT");
                    telemetry.update();
                } else { //LEFT
                    telemetry.addLine("Detected Prop! POSITION:LEFT");
                    telemetry.update();
                }
            }
        }

           if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) { // CENTER
          //  if(propPos == 1){
                telemetry.addLine("Detected Prop! Position:CENTER");
                telemetry.update();
                BlueCenterProp(drive, 35, startPose);

            } else if (propDetector.centerOfProp.x >= 200) { //right
        // (propPos == 3){
                telemetry.addLine("Detected Prop! POSITION:RIGHT");
                telemetry.update();
                BlueRightProp(drive, 35, startPose);
            } else { //LEFT
                telemetry.addLine("Detected Prop! POSITION:LEFT");
                telemetry.update();
                BlueLeftProp(drive, 35, startPose);
            }

            if (isStopRequested()) return;
            stop();
        }



    public void BlueRightProp(Drive drive, int slowerVelocity, Pose2d start){
            drive.setPoseEstimate(start);
            TrajectorySequence BlueRightProp = drive.trajectorySequenceBuilder(start)
                    .lineTo(new Vector2d(12,34))
                    .turn(Math.toRadians(-90))
                    .build();
            TrajectorySequence BlueRightPropToBackdrop = drive.trajectorySequenceBuilder(BlueRightProp.end())
                    .setReversed(true)
                    .lineTo(new Vector2d(49, 22.25))
                    .build();
        Trajectory PullAway = drive.trajectoryBuilder(BlueRightPropToBackdrop.end())
                .forward(9)
                .build();
            TrajectorySequence BlueRightBackDropToPark = drive.trajectorySequenceBuilder(PullAway.end())
                    .strafeRight(30)
                    .build();


        drive.followTrajectorySequence(BlueRightProp);
        drive.intake.setPower(.4);
        sleep(1000);
        drive.intake.setPower(-.25);
        drive.followTrajectorySequence(BlueRightPropToBackdrop);
        drive.intake.setPower(0);
        extendSlides(drive,.8,200);
        drive.pixelScore.setPosition(.559);
        sleep(1000);
        //added increase for slides so robot can pull away from backdrop
        extendSlidesPullAway(drive,.5,300);
        drive.followTrajectory(PullAway);
        drive.pixelScore.setPosition(0);
        sleep(1000);
        retractSlides(drive,.8);
        drive.followTrajectorySequence(BlueRightBackDropToPark);


    }
    public void BlueLeftProp(Drive drive, int slowerVelocity, Pose2d start){
        drive.setPoseEstimate(start);
        TrajectorySequence BlueLeftProp = drive.trajectorySequenceBuilder(start)
                .lineTo(new Vector2d(25, 45))
                .build();
        TrajectorySequence BlueLeftPropToBackDrop = drive.trajectorySequenceBuilder(BlueLeftProp.end())
               .back(7)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(49, 32))
                .build();
        Trajectory PullAway = drive.trajectoryBuilder(BlueLeftPropToBackDrop.end())
                .forward(9)
                .build();
        TrajectorySequence BlueLeftBackDropToPark = drive.trajectorySequenceBuilder(PullAway.end())
                .strafeRight(20)
                .build();
        drive.followTrajectorySequence(BlueLeftProp);
        drive.intake.setPower(.25);
        sleep(1000);
        drive.intake.setPower(-.25);
        drive.followTrajectorySequence(BlueLeftPropToBackDrop);
        drive.intake.setPower(0);
        extendSlides(drive,.8,200);
        drive.pixelScore.setPosition(.6);
        sleep(1000);
        //added increase for slides so robot can pull away from backdrop
        extendSlidesPullAway(drive,.5,300);
        drive.followTrajectory(PullAway);
        drive.pixelScore.setPosition(0);
        sleep(1000);
        retractSlides(drive,.8);
        drive.followTrajectorySequence(BlueLeftBackDropToPark);

    }
    public void BlueCenterProp(Drive drive, int slowerVelocity, Pose2d start){
        drive.setPoseEstimate(start);
        TrajectorySequence BlueCenterProp = drive.trajectorySequenceBuilder(start)
//                .splineTo(new Vector2d(10.97, 34.51), Math.toRadians(270))
                .lineTo(new Vector2d(14.67, 35.03))
                .build();
        TrajectorySequence BlueCenterPropToBackDrop = drive.trajectorySequenceBuilder(BlueCenterProp.end())
                .waitSeconds(.5)
                .back(6)
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(49, 30))
                .build();

        Trajectory PullAway = drive.trajectoryBuilder(BlueCenterPropToBackDrop.end())
                .forward(9)
                .build();

        TrajectorySequence BlueCenterBackDropToPark = drive.trajectorySequenceBuilder(PullAway.end())
                .strafeRight(28)
                .build();
        drive.followTrajectorySequence(BlueCenterProp);
        drive.intake.setPower(.25);
        sleep(1000);
        drive.intake.setPower(-.25);
        drive.followTrajectorySequence(BlueCenterPropToBackDrop);
        drive.intake.setPower(0);
        //lowered to 200
        extendSlides(drive,.8,200);
        drive.pixelScore.setPosition(.6);
        sleep(1000);
        //added increase for slides so robot can pull away from backdrop
        extendSlidesPullAway(drive,.5,300);
        drive.followTrajectory(PullAway);
        drive.pixelScore.setPosition(0);
        sleep(1000);
        retractSlides(drive,.8);
        drive.followTrajectorySequence(BlueCenterBackDropToPark);
    }

    public void extendSlides(Drive drive, double speed, int position) {
        drive.rightSlide.setTargetPosition(position);
        drive.leftSlide.setTargetPosition(position);

        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.rightSlide.setPower(speed);
        drive.leftSlide.setPower(speed);
    }

    public void extendSlidesPullAway(Drive drive, double speed, int position){
        drive.rightSlide.setTargetPosition(position);

        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.rightSlide.setPower(speed);
        while(drive.rightSlide.isBusy()) {
            drive.leftSlide.setPower(drive.rightSlide.getPower());
            telemetry.addLine(String.valueOf(drive.rightSlide.getPower()));
            telemetry.addLine(String.valueOf(drive.leftSlide.getPower()));
            telemetry.update();
        }
        drive.leftSlide.setPower(0);
    }

    public void retractSlides(Drive drive, double speed){
        drive.rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.rightSlide.setPower(-speed);
        drive.leftSlide.setPower(-speed);
    }
}