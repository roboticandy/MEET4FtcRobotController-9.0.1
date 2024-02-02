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
public class CleanBlueBackStageMain extends LinearOpMode {

    @Override
    public void runOpMode() {
        TrajectorySequence[] currentTrajSeq = new TrajectorySequence[4];
        int propPos;
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
        Pose2d startPose =      new Pose2d(12,     63,Math.toRadians(270));
        Vector2d leftSpike =      new Vector2d(25,     45);
        Vector2d centerSpike =    new Vector2d(14.67,     35.03);
        Vector2d rightSpike =     new Vector2d(12,   34);
        Vector2d leftBackDrop =   new Vector2d(49,     38  );
        Vector2d centerBackDrop = new Vector2d(49,     29  );
        Vector2d rightBackDrop =  new Vector2d(49,     22.25);

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
                propPos = 2;
                telemetry.addLine("Detected Prop! Position:CENTER");
                telemetry.update();
               currentTrajSeq[0] = BlueProp(drive, 30,startPose,centerSpike,180, propPos);
               currentTrajSeq[1] = BlueProp2BD(drive, 30,currentTrajSeq[0].end(),centerBackDrop, propPos);
               currentTrajSeq[2] = StraightForward(drive, 30,currentTrajSeq[1].end(),9);

            } else if (propDetector.centerOfProp.x >= 200) { //right
               propPos = 3;
                telemetry.addLine("Detected Prop! POSITION:RIGHT");
                telemetry.update();
               currentTrajSeq[0] = BlueProp(drive, 30,startPose,rightSpike,180, propPos);
               currentTrajSeq[1] = BlueProp2BD(drive, 30,currentTrajSeq[0].end(),rightBackDrop, propPos);
               currentTrajSeq[2] = StraightForward(drive, 30,currentTrajSeq[1].end(),9);
            } else { //LEFT
               propPos = 1;
                telemetry.addLine("Detected Prop! POSITION:LEFT");
                telemetry.update();
               currentTrajSeq[0] = BlueProp(drive, 30,startPose,leftSpike,270, propPos);
               currentTrajSeq[1] = BlueProp2BD(drive, 30,currentTrajSeq[0].end(),leftBackDrop, propPos);
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
        sleep(500);
            if (isStopRequested()) return;
            stop();
        }



    public TrajectorySequence BlueProp(Drive drive, int slowerVelocity, Pose2d start, Vector2d endVector, int endHeading, int propPos) {
        drive.setPoseEstimate(start);
        TrajectorySequence BluePropSeq = null;
        if(propPos == 1) {
            //LEFT
            BluePropSeq = drive.trajectorySequenceBuilder(start)
                    .lineTo(endVector)
                    .build();
        } else if (propPos == 2){
            //CENTER
            BluePropSeq = drive.trajectorySequenceBuilder(start)
                    .splineTo(endVector, Math.toRadians(270))
                    .build();
        } else{
            //RIGHT
            BluePropSeq = drive.trajectorySequenceBuilder(start)
//                    .lineTo(new Vector2d(12,34))
                    .lineTo(endVector)
                    .turn(Math.toRadians(-90))
//                   .lineTo(endVector)
                    .build();
        }
        return BluePropSeq;
    }
    public TrajectorySequence BlueProp2BD(Drive drive, int slowerVelocity, Pose2d seqStart, Vector2d endVector, int position) {
        if(position == 3){
            TrajectorySequence BlueRightPropToBackdrop = drive.trajectorySequenceBuilder(seqStart)
                    .setReversed(true)
                    .lineTo(endVector)
                    .build();
            return BlueRightPropToBackdrop;
        }else{
            TrajectorySequence BlueRightPropToBackdrop = drive.trajectorySequenceBuilder(seqStart)
                    .turn(Math.toRadians(-90))
                    .setReversed(true)
                    .lineTo(endVector)
                    .build();
            return BlueRightPropToBackdrop;
        }

    }
    public TrajectorySequence StraightForward(Drive drive, int slowerVelocity, Pose2d seqStart, int distance){
        TrajectorySequence PullAwaySeq = drive.trajectorySequenceBuilder(seqStart)
                    .forward(distance)
                    .build();
        return PullAwaySeq;
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