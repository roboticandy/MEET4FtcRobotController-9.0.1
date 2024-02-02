package org.firstinspires.ftc.teamcode.Actual;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class CleanRedBackStageMain extends LinearOpMode {

    @Override
    public void runOpMode() {
        OpenCvWebcam webcam;
        TrajectorySequence[] currentTrajSeq = new TrajectorySequence[4];
        int propPos;
        Drive drive = new Drive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        RedCircleDetector propDetector = new RedCircleDetector();
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

        Pose2d startPose = new Pose2d(12, -63.00, Math.toRadians(90));
        Vector2d leftSpike = new Vector2d(14, -36.16);
        Vector2d centerSpike = new Vector2d(16,-34.71);
        Vector2d rightSpike = new Vector2d(23.33,-44.99);
        Vector2d leftBackDrop = new Vector2d(58, -36);
        Vector2d centerBackDrop = new Vector2d(57, -40);
        Vector2d rightBackDrop = new Vector2d(57, -46);


        while (!isStarted()) {
            if (propDetector.centerOfProp.x > 1 && propDetector.centerOfProp.x < 200) { // center
                telemetry.addLine("Detected Prop! Position:CENTER");
            } else if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x >= 200) { //right
                telemetry.addLine("Detected Prop! POSITION:RIGHT");
            } else { //left
                telemetry.addLine("Detected Prop! POSITION:LEFT");
                telemetry.addData("Position L/R :", drive.leftSlide.getCurrentPosition() + "/" + drive.rightSlide.getCurrentPosition());
            }
            telemetry.update();
        }
            if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) { // center
                propPos = 2;
                telemetry.addLine("Detected Prop! Position:CENTER");
                telemetry.update();
                currentTrajSeq[0] = RedProp(drive,30,startPose,centerSpike,90,propPos);
                currentTrajSeq[1] = RedProp2BD(drive, 30,currentTrajSeq[0].end(),centerBackDrop);
                currentTrajSeq[2] = StraightForward(drive, 30,currentTrajSeq[1].end(),9);

            } else if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x >= 200) { //right
                propPos = 3;
                telemetry.addLine("Detected Prop! POSITION:RIGHT");
                telemetry.update();
                currentTrajSeq[0] = RedProp(drive, 30,startPose,rightSpike,180, propPos);
                currentTrajSeq[1] = RedProp2BD(drive, 30,currentTrajSeq[0].end(),rightBackDrop);
                currentTrajSeq[2] = StraightForward(drive, 30,currentTrajSeq[1].end(),9);
            } else { //left
                propPos = 1;
                telemetry.addLine("Detected Prop! POSITION:LEFT");
                telemetry.update();
                currentTrajSeq[0] = RedProp(drive,30,startPose,leftSpike,90,propPos);
                currentTrajSeq[1] = RedProp2BD(drive, 30,currentTrajSeq[0].end(),leftBackDrop);
                currentTrajSeq[2] = StraightForward(drive, 30,currentTrajSeq[1].end(),9);
            }
        drive.followTrajectorySequence(currentTrajSeq[0]);
        drive.intake.setPower(.25);
        sleep(1000);
        drive.intake.setPower(-.25);
        drive.followTrajectorySequence(currentTrajSeq[1]);
        drive.intake.setPower(0);
        extendSlides(drive,.8,200);
        drive.pixelScore.setPosition(.6);
        sleep(1000);
        //added increase for slides so robot can pull away from backdrop
        extendSlidesPullAway(drive,.5,300);
        drive.followTrajectorySequence(currentTrajSeq[2]);
        drive.pixelScore.setPosition(0);
        sleep(1000);
        retractSlides(drive,.8);
            if (isStopRequested()) {
            }

    }
        public TrajectorySequence RedProp(Drive drive, int slowerVelocity, Pose2d start, Vector2d endVector, int endHeading, int propPos){
        drive.setPoseEstimate(start);
        TrajectorySequence RedPropSeq = null;
        if(propPos ==1){
            // CENTER
            RedPropSeq = drive.trajectorySequenceBuilder(start)
                    .lineTo(endVector)
                    .build();
        }
        else if(propPos == 2) {
            //right
            RedPropSeq = drive.trajectorySequenceBuilder(start)
                    .lineTo(endVector)
                    .build();
        } else {
            // left
            RedPropSeq = drive.trajectorySequenceBuilder(start)
                    .lineTo(new Vector2d(18.36, -36.16))
                    .turn(Math.toRadians(90))
                    .lineTo(endVector)
                    .build();
        }
        return RedPropSeq;
        }
    public TrajectorySequence RedProp2BD(Drive drive, int slowerVelocity, Pose2d seqStart, Vector2d endVector) {
        TrajectorySequence RedPropToBackDrop = drive.trajectorySequenceBuilder(seqStart)
                .setReversed(true)
                .lineTo(endVector)
                .build();
        return RedPropToBackDrop;
    }
    public TrajectorySequence StraightForward(Drive drive, int slowerVelocity, Pose2d seqStart, int distance){
        TrajectorySequence PullAwaySeq = drive.trajectorySequenceBuilder(seqStart)
                .forward(distance)
                .build();
        return PullAwaySeq;
    }


    public void extendSlides(Drive drive, double speed, int position){
//        Drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                Drive.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        System.out.println("Current Position: "+drive.rightSlide.getCurrentPosition());
        drive.pixelScore.setPosition(0);
        drive.rightSlide.setTargetPosition(position);
//                Drive.leftSlide.setTargetPosition(position);
        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        System.out.println("Right Is Busy: "+drive.rightSlide.isBusy());
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.rightSlide.setPower(speed);
        while((drive.rightSlide.getCurrentPosition() < position || drive.rightSlide.isBusy()) && opModeIsActive()) {
//            System.out.println("Right Is Busy: "+Drive.rightSlide.isBusy());
//            System.out.println("Current Slide Position: "+Drive.rightSlide.getCurrentPosition());
//            System.out.println("Current Pixel Door Position: "+Drive.pixelScore.getPosition());
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

