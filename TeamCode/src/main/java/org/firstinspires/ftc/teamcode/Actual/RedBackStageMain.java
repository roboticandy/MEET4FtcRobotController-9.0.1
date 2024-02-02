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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Config

@Autonomous
public class RedBackStageMain extends LinearOpMode {
    public static int propPos = 1;
    @Override
    public void runOpMode() {
        OpenCvWebcam webcam;
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
        Vector2d redLeftBB, redCenterBB, redRightBB;

        redLeftBB = new Vector2d(58, -36);
        redCenterBB = new Vector2d(57, -40);
        redRightBB = new Vector2d(57, -46);


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
            telemetry.addLine(String.valueOf(drive.rightSlide.getTargetPositionTolerance()));
            telemetry.addLine(String.valueOf(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            telemetry.addData("X : Y ", propDetector.centerOfProp.x + ":" + propDetector.centerOfProp.y);
            telemetry.update();
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

            if (isStopRequested()) {
            }

    }
    public void RedLeftProp(Drive drive, int slowerVelocity, Pose2d start, Vector2d boardPos){
        drive.setPoseEstimate(start);
        TrajectorySequence RedLeftProp = drive.trajectorySequenceBuilder(start)
                .lineTo(new Vector2d(18.36, -36.16), Drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(14, -36.16), Drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence RedLeftPropToBackdrop = drive.trajectorySequenceBuilder(RedLeftProp.end())
                .setReversed(true)
                .lineTo(boardPos, Drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),

                        Drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory PullAway = drive.trajectoryBuilder(RedLeftPropToBackdrop.end())
                .forward(9)
                .build();

        TrajectorySequence RedBackdroptoPark = drive.trajectorySequenceBuilder(PullAway.end())
                .strafeLeft(26)
                .build();
        drive.followTrajectorySequence(RedLeftProp);
        drive.intake.setPower(.4);
        sleep(1000);
        drive.intake.setPower(-.25);
        drive.followTrajectorySequence(RedLeftPropToBackdrop);
        drive.intake.setPower(0);
        extendSlides(drive, .8,200);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(1000);
        //added increase for slides so robot can pull away from backdrop
        extendSlidesPullAway(drive,.5,300);
        drive.followTrajectory(PullAway);
        drive.pixelScore.setPosition(0);
        sleep(1000);
        retractSlides(drive,.8);
        drive.followTrajectorySequence(RedBackdroptoPark);


    }
            public void RedCenterProp(Drive drive, int slowerVelocity, Pose2d start, Vector2d boardPos){
                drive.setPoseEstimate(start);
                TrajectorySequence RedCenter = drive.trajectorySequenceBuilder(start)
                        .splineTo(new Vector2d(16, -34.71), Math.toRadians(88.88), Drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                Drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                TrajectorySequence RedLineToBackDrop = drive.trajectorySequenceBuilder(RedCenter.end())
                        .back(9)
                        .turn(Math.toRadians(90))
                        .lineTo(boardPos, Drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                Drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                Trajectory PullAway = drive.trajectoryBuilder(RedLineToBackDrop.end())
                        .forward(9)
                        .build();
                TrajectorySequence RedCenterBackDropToPark = drive.trajectorySequenceBuilder(PullAway.end())
                        .strafeLeft(21)
                        .build();


                drive.followTrajectorySequence(RedCenter);
                drive.intake.setPower(.25);
                sleep(1000);
                drive.intake.setPower(-.25);
                drive.followTrajectorySequence(RedLineToBackDrop);
                drive.intake.setPower(0);
                extendSlides(drive,.8,200);
                while (drive.rightSlide.isBusy()) ;
                drive.pixelScore.setPosition(.559);
                sleep(1000);
                //added increase for slides so robot can pull away from backdrop
                extendSlidesPullAway(drive,.5,300);
                drive.followTrajectory(PullAway);
                drive.pixelScore.setPosition(0);
                sleep(250);
                retractSlides(drive,.8);
                drive.followTrajectorySequence(RedCenterBackDropToPark);
                }

    public void RedRightProp(Drive drive, int slowerVelocity, Pose2d start, Vector2d boardPos){
        drive.setPoseEstimate(start);
        TrajectorySequence RedRightProp = drive.trajectorySequenceBuilder(start)
//                .splineTo(new Vector2d(23.33, -45.52), Math.toRadians(90.00),SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                .lineTo(new Vector2d(23.33, -44.99), Drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
        Drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence RedRightProptoBackdrop = drive.trajectorySequenceBuilder(RedRightProp.end())
                .back(10)
                .turn(Math.toRadians(90))
                .lineTo(boardPos, Drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory PullAway = drive.trajectoryBuilder(RedRightProptoBackdrop.end())
                .forward(9)
                .build();
        TrajectorySequence RedRightBackdropToPark = drive.trajectorySequenceBuilder(PullAway.end())
                .strafeLeft(19)
                .build();
        drive.followTrajectorySequence(RedRightProp);
        drive.intake.setPower(.25);
        sleep(1000);
        drive.intake.setPower(-.25);
//                while(Drive.rightSlide.isBusy() && Drive.leftSlide.isBusy() && opModeIsActive()){
//                    telemetry.addData("Position L/R :", Drive.leftSlide.getCurrentPosition()+"/"+Drive.rightSlide.getCurrentPosition());
//                    telemetry.update();
//                }
        drive.followTrajectorySequence(RedRightProptoBackdrop);
        drive.intake.setPower(0);
        extendSlides(drive, .8,200);
        while (drive.rightSlide.isBusy());
        drive.pixelScore.setPosition(.559);
        sleep(500);
        //added increase for slides so robot can pull away from backdrop
        extendSlidesPullAway(drive,.5,300);
        drive.followTrajectory(PullAway);
        drive.pixelScore.setPosition(0);
        sleep(250);
        retractSlides(drive,.8);
        drive.followTrajectorySequence(RedRightBackdropToPark);

    }

    public void extendSlides(Drive drive, double speed, int position) {
        drive.rightSlide.setTargetPosition(position);
        drive.leftSlide.setTargetPosition(position);

        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.rightSlide.setPower(speed);
        drive.leftSlide.setPower(speed);
    }
//    public void extendSlides(Drive drive, double speed, int position){
////        Drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////                Drive.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        System.out.println("Current Position: "+drive.rightSlide.getCurrentPosition());
//        drive.pixelScore.setPosition(0);
//        drive.rightSlide.setTargetPosition(position);
////                Drive.leftSlide.setTargetPosition(position);
//        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        System.out.println("Right Is Busy: "+drive.rightSlide.isBusy());
//        drive.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        drive.rightSlide.setPower(speed);
//        while((drive.rightSlide.getCurrentPosition() < position || drive.rightSlide.isBusy()) && opModeIsActive()) {
////            System.out.println("Right Is Busy: "+Drive.rightSlide.isBusy());
////            System.out.println("Current Slide Position: "+Drive.rightSlide.getCurrentPosition());
////            System.out.println("Current Pixel Door Position: "+Drive.pixelScore.getPosition());
//            drive.leftSlide.setPower(drive.rightSlide.getPower());
//            telemetry.addLine(String.valueOf(drive.rightSlide.getPower()));
//            telemetry.addLine(String.valueOf(drive.leftSlide.getPower()));
//            telemetry.update();
//        }
//        drive.leftSlide.setPower(0);
//    }
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

