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

@Autonomous
@Disabled
@Config
public class RedFrontStageMain extends LinearOpMode {
    public static int propPos = 1;
    int slowerVelocity = 35;
    @Override
    public void runOpMode() throws InterruptedException {
        OpenCvWebcam webcam;
        Drive drive = new Drive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        CircleDetector propDetector = new CircleDetector();
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
        Pose2d startPose = new Pose2d(-37, -63.00, Math.toRadians(90));
        Vector2d redLeftBB, redCenterBB, redRightBB;

        redLeftBB = new Vector2d(55, -32);
        redCenterBB = new Vector2d(55, -42);
        redRightBB = new Vector2d(55, -46);
        while (!isStarted()) {
            if (propDetector.centerOfProp.x > 1 && propDetector.centerOfProp.x < 200) { // center
                telemetry.addLine("Detected Prop! Position:CENTER");
            } else if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x >= 200) { //right
                telemetry.addLine("Detected Prop! POSITION:RIGHT");
            } else { //left
                telemetry.addLine("Detected Prop! POSITION:LEFT");
                telemetry.addData("Est Start: ", startPose);
                telemetry.addData("Position L/R :", drive.leftSlide.getCurrentPosition() + "/" + drive.rightSlide.getCurrentPosition());
            }
        }
        if (propDetector.centerOfProp.x > 1 && propDetector.centerOfProp.x < 200) { // center
            telemetry.addLine("Detected Prop! Position:CENTER");
            telemetry.update();
            RedCenterProp(drive, 35, startPose, redCenterBB);

        } else if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x >= 200) { //right
            telemetry.addLine("Detected Prop! POSITION:RIGHT");
            telemetry.update();
            RedRightProp(drive, 35, startPose, redRightBB);
        } else { //left
            telemetry.addLine("Detected Prop! POSITION:LEFT");
            telemetry.update();
            RedLeftProp(drive, 35, startPose, redLeftBB);
        }

        if (isStopRequested()) return;
        if (isStopRequested()) {
        }

    }
    public void RedRightProp(Drive drive, int slowerVelocity, Pose2d start, Vector2d boardPos){
        drive.setPoseEstimate(start);
        TrajectorySequence RedRightProp = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(new Pose2d(-35.46, -30.98, Math.toRadians(0.00)))
                .build();
        TrajectorySequence RedRightPropToBackDrop = drive.trajectorySequenceBuilder(RedRightProp.end())
                .back(10)
                .turn(Math.toRadians(180.00))
                .lineTo(new Vector2d(-36.32, -9.46))
                .lineTo(new Vector2d(16.97, -10.90))
                .lineTo(boardPos)
                .build();
        Trajectory PullAway = drive.trajectoryBuilder(RedRightPropToBackDrop.end())
                .forward(9)
                .build();
        TrajectorySequence RedRightBackDropToPark = drive.trajectorySequenceBuilder(PullAway.end())
                .strafeLeft(19)
                .build();
        drive.followTrajectorySequence(RedRightProp);
        drive.intake.setPower(.5);
        sleep(1000);
        drive.intake.setPower(-.25);
        drive.followTrajectorySequence(RedRightPropToBackDrop);
        drive.intake.setPower(0);
        extendSlides(drive, .8,200);
        drive.pixelScore.setPosition(.6);
        sleep(1000);
        //added increase for slides so robot can pull away from backdrop
        extendSlidesPullAway(drive,.5,300);
        drive.followTrajectory(PullAway);
        drive.pixelScore.setPosition(0);
        sleep(1000);
        retractSlides(drive,.8);
        drive.followTrajectorySequence(RedRightBackDropToPark);
    }
    public void RedCenterProp(Drive drive, int slowerVelocity, Pose2d start, Vector2d boardPos){
        drive.setPoseEstimate(start);
        TrajectorySequence RedCenterProp =drive.trajectorySequenceBuilder(start)
                .lineTo(new Vector2d(-36,-34))
                .build();
        TrajectorySequence RedCenterPropToBackdrop = drive.trajectorySequenceBuilder(RedCenterProp.end())
                .lineTo(new Vector2d(-58.57, -33.73))
                .lineTo(new Vector2d(-58.42, -13.79))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(11.77, -13.79))
                .lineTo(boardPos)
                .build();
        Trajectory PullAway = drive.trajectoryBuilder(RedCenterPropToBackdrop.end())
                .forward(9)
                .build();
        TrajectorySequence RedCenterPark = drive.trajectorySequenceBuilder(PullAway.end())
                .strafeLeft(21)
        .build();

        drive.followTrajectorySequence(RedCenterProp);
        drive.intake.setPower(.25);
        sleep(1000);
        drive.intake.setPower(-.25);
        drive.followTrajectorySequence(RedCenterPropToBackdrop);
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
        drive.followTrajectorySequence(RedCenterPark);
    }
    public void RedLeftProp(Drive drive, int slowerVelocity, Pose2d start, Vector2d boardPos){
        TrajectorySequence RedLeftProp = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(new Pose2d(-35.91, -31.13, Math.toRadians(180.00)))
                .build();
        TrajectorySequence RedLeftPropToBackdrop = drive.trajectorySequenceBuilder(RedLeftProp.end())
                .lineTo(new Vector2d(-36.32, -9.46))
                .lineTo(new Vector2d(16.97, -10.90))
                .lineTo(boardPos)
                .build();
        Trajectory PullAway = drive.trajectoryBuilder(RedLeftPropToBackdrop.end())
                .forward(9)
                .build();
        TrajectorySequence RedLeftPark = drive.trajectorySequenceBuilder(PullAway.end())
                .strafeLeft(26)
                .build();
        drive.followTrajectorySequence(RedLeftProp);
        drive.intake.setPower(.4);
        sleep(1000);
        drive.intake.setPower(-.25);
        drive.followTrajectorySequence(RedLeftPropToBackdrop);
        drive.intake.setPower(0);

        extendSlides(drive, .8,200);
        drive.pixelScore.setPosition(.6);
        sleep(1000);
        //added increase for slides so robot can pull away from backdrop
        extendSlidesPullAway(drive,.5,300);
        drive.followTrajectory(PullAway);
        drive.pixelScore.setPosition(0);
        sleep(1000);
        retractSlides(drive,.8);
        drive.followTrajectorySequence(RedLeftPark);

    }




    public void extendSlides(Drive drive, double speed, int position){
//        Drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                Drive.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.rightSlide.setTargetPosition(position);
//                Drive.leftSlide.setTargetPosition(position);

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

