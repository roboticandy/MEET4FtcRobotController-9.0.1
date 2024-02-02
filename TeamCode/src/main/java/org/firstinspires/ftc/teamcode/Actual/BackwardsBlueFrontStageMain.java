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
@Disabled
public class BackwardsBlueFrontStageMain extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(-35, 63, Math.toRadians(90));
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

          // if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) { // CENTER
           if(propPos == 1){
                telemetry.addLine("Detected Prop! Position:CENTER");
                telemetry.update();
                BlueCenterProp(drive, 35, startPose);

            } else if  //(propDetector.centerOfProp.x >= 200) { //right
         (propPos == 3){
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
        drive.intake.setPower(.25);
        sleep(1000);
        drive.intake.setPower(-.25);
        drive.followTrajectorySequence(BlueRightPropToBackdrop);
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
        drive.followTrajectorySequence(BlueRightBackDropToPark);


    }
    public void BlueLeftProp(Drive drive, int slowerVelocity, Pose2d start){
        drive.setPoseEstimate(start);
        TrajectorySequence BlueLeftProp = drive.trajectorySequenceBuilder(start)
//                .lineToLinearHeading(new Pose2d(-36.16, 29.0,Math.toRadians(0.00)))
                .lineTo(new Vector2d(-43,35))
                .turn(Math.toRadians(90))
                .forward(5)
                .build();
        TrajectorySequence BlueLeftPropToBackDrop = drive.trajectorySequenceBuilder(BlueLeftProp.end())
                .lineTo(new Vector2d(-34.01, 7.01))
                .turn(Math.toRadians(180))
                .lineTo(new Vector2d(13.94, 14.08))
                .lineTo(new Vector2d(48, 42.52))
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
//                .lineTo(new Vector2d(-37.71, 15))
                .addDisplacementMarker(35,() -> {
                    drive.intake.setPower(-.4);
                })
                .addDisplacementMarker(52,() -> {
                    drive.intake.setPower(0);
                })
                .lineTo(new Vector2d(-37.71, 9))
//                .turn(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-52, 7.5, Math.toRadians(180.00)), Math.toRadians(180.00))

//                .build();
//        TrajectorySequence ExtraPixel = drive.trajectorySequenceBuilder(BlueCenterProp.end())
//                .lineTo(new Vector2d(-53.50, 7.5))
                .lineTo(new Vector2d(-47.50,7.5))
                .addDisplacementMarker(67,() -> {
                    drive.dropdown.setPosition(.410);
                })
                .addDisplacementMarker(72,() -> {
                    drive.intake.setPower(1);
                    drive.dropdown.setPosition(.300);
                })
                .lineTo(new Vector2d(-50, 7.5))
                .lineTo(new Vector2d(-36,7.5))
                .build();

//        TrajectorySequence ExtraPixel = drive.trajectorySequenceBuilder(BlueCenterProp.end())
//                .back(6)
//                .turn(Math.toRadians(90))
//                .lineTo(new Vector2d(-50.5,10))
////                .forward(12)
//                .build();
//        TrajectorySequence LurgeForward = drive.trajectorySequenceBuilder(ExtraPixel.end())
//                .forward(.5)
//                .build();
        TrajectorySequence BlueCenterPropToBackDrop = drive.trajectorySequenceBuilder(BlueCenterProp.end())
                .setReversed(true)
                .lineTo(new Vector2d(25.94, 8))
                .lineTo(new Vector2d(58.20, 32))
                .build();

        Trajectory PullAway = drive.trajectoryBuilder(BlueCenterPropToBackDrop.end())
                .forward(9)
                .build();



        TrajectorySequence DriveBack = drive.trajectorySequenceBuilder(PullAway.end())
                .strafeLeft(30)
                .lineTo(new Vector2d(-53.00, 7.5))
                .lineTo(new Vector2d(-48.00,7.5))
                .addDisplacementMarker(132,() -> {
                    drive.dropdown.setPosition(.380);
                })
                .addDisplacementMarker(135,() -> {
                    drive.intake.setPower(1);
                    drive.dropdown.setPosition(.300);
                })
                .lineTo(new Vector2d(-50, 7.5))
                .lineTo(new Vector2d(-36,7.5))
                .strafeLeft(4)
                .build();

        TrajectorySequence BlueCenterBackDropToPark = drive.trajectorySequenceBuilder(DriveBack.end())
                .strafeRight(28)
                .build();
        drive.followTrajectorySequence(BlueCenterProp);
//        drive.intake.setPower(.25);
//        sleep(1000);
//        drive.intake.setPower(-.25);
//        sleep(500);
//        drive.intake.setPower(0);
//        drive.followTrajectorySequence(ExtraPixel);
//        drive.intake.setPower(1);
//        double dropdownValue = .7;
//        while(dropdownValue > .38){
//            drive.dropdown.setPosition(dropdownValue);
//            dropdownValue = dropdownValue - .01;
//            sleep(100);
//        }
//        drive.followTrajectorySequence(LurgeForward);
  //      dropdownValue = .7;
        sleep(2000);
        drive.followTrajectorySequence(BlueCenterPropToBackDrop);
        drive.intake.setPower(0);
        extendSlides(drive,.8,200);
        drive.pixelScore.setPosition(.6);
        sleep(1000);
        drive.followTrajectory(PullAway);
        drive.pixelScore.setPosition(0);
        retractSlides(drive,.8);
        drive.dropdown.setPosition(1);
        drive.followTrajectorySequence(DriveBack);
//        while(dropdownValue > .38){
//            drive.dropdown.setPosition(dropdownValue);
//            dropdownValue = dropdownValue - .01;
//            sleep(100);
//        }
        drive.followTrajectorySequence(BlueCenterPropToBackDrop);
        drive.intake.setPower(0);
        extendSlides(drive,.8,200);
        drive.pixelScore.setPosition(.6);
        drive.followTrajectory(PullAway);
//        drive.intake.setPower(-.25);
//        drive.followTrajectorySequence(BlueCenterPropToBackDrop);
//        drive.intake.setPower(0);
//        //lowered to 200
//        extendSlides(drive,.8,200);
//        drive.pixelScore.setPosition(.6);
//        sleep(1000);
//        //added increase for slides so robot can pull away from backdrop
//        extendSlidesPullAway(drive,.5,300);
//        drive.followTrajectory(PullAway);
//        drive.pixelScore.setPosition(0);
//        sleep(1000);
//        retractSlides(drive,.8);
//        drive.followTrajectorySequence(BlueCenterBackDropToPark);
    }

    public void extendSlides(Drive drive, double speed, int position){
        drive.pixelScore.setPosition(0);
//        Drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.rightSlide.setTargetPosition(position);

        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.rightSlide.setPower(speed);
        while((drive.rightSlide.getCurrentPosition() < position || drive.rightSlide.isBusy()) && opModeIsActive()) {
            drive.leftSlide.setPower(drive.rightSlide.getPower());
            telemetry.addLine(String.valueOf(drive.rightSlide.getPower()));
            telemetry.addLine(String.valueOf(drive.leftSlide.getPower()));
            telemetry.update();
        }
        drive.leftSlide.setPower(0);
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