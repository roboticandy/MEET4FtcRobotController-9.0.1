package org.firstinspires.ftc.teamcode.Actual;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Config
@Disabled
public class RedFrontStage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpenCvWebcam webcam;
        Drive drive = new Drive(hardwareMap);
        TrajectorySequence[] currentTrajSeq = new TrajectorySequence[4];
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

        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(90));

        while (!isStarted()) {

            {
                if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) { // CENTER
                    drive.ledLightAdjust(31,0,255,0);
                    telemetry.addLine("Detected Prop! Position:CENTER");
                    telemetry.update();
                } else if (propDetector.centerOfProp.x >= 200) {
                    drive.ledLightAdjust(31,0,0,255);
                    telemetry.addLine("Detected Prop! POSITION:RIGHT");
                    telemetry.update();
                } else { //LEFT
                    drive.ledLightAdjust(31,255,0,0);
                    telemetry.addLine("Detected Prop! POSITION:LEFT");
                    telemetry.update();
                }
            }
        }

        if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) {
            RedCenterProp(drive,35,startPose);
        } else if (propDetector.centerOfProp.x >= 200) { //right
            // (propPos == 3){
            RedRightProp(drive, 30, startPose);
        } else {
            RedLeftProp(drive,35,startPose);
        }
        stop();
        if (isStopRequested()) return;
        stop();
    }

    public void RedRightProp(Drive drive, int slowerVelocity, Pose2d start) {
        ElapsedTime timer = new ElapsedTime();
        drive.setPoseEstimate(start);
        TrajectorySequence redRightProp = drive.trajectorySequenceBuilder(start)
                //pull away turn
                .lineToLinearHeading(new Pose2d(-40, -41, Math.toRadians(180)))
                //backup to spike
                .lineTo(new Vector2d(-25, -41))
                //crab walk to stack
                .lineTo(new Vector2d(-49.5, -15.75))
                //back away from stack
                .lineTo(new Vector2d(-44,-15.75))
                //drive towards stack
                .lineTo(new Vector2d(-52, -15.75))
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -15.75, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(57, -53, Math.toRadians(180.00)))
//                release purple
                .addDisplacementMarker(39,() -> {
                    drive.releasePurplePixel();
                })
                //deploy dropdown
                .addDisplacementMarker(72.25,() -> {
                    timer.reset();
                    timer.startTime();
                    drive.dropdown.setPosition(.399);
                    drive.intake.setPower(.5);
                    while(timer.time() < 1){
                        drive.setMotorPowers(0,0,0,0);
                    }
                })
                //retract dropdown / intake full speed
                .addDisplacementMarker(74,() -> {
                    drive.dropdown.setPosition(1);
                    drive.intake.setPower(1);
                })
                .build();
        TrajectorySequence comeBackToStack = drive.trajectorySequenceBuilder(redRightProp.end())
                //to center gate path
                .lineToLinearHeading(new Pose2d(30.84, -17, Math.toRadians(180.00)))
                //to pixel stack
                .lineTo(new Vector2d(-51, -15.75))
                //back away from stack
                .lineTo(new Vector2d(-48,-15.75))
                //drive towards stack
                .lineTo(new Vector2d(-52, -15.75))
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -15.75, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(56, -44, Math.toRadians(180.00)))

                    // retract slides
                .addDisplacementMarker(6,() -> {
                    retractSlides(drive, .5);
                })
                .addDisplacementMarker(50,() -> {
                    stopResetSlides(drive);
                })
                //deploy dropdown
                .addDisplacementMarker(122,()->{
                    drive.dropdown.setPosition(.385);
                    drive.intake.setPower(.5);
                })
                //intake at full speed for 3rd and 4th pixel
                .addDisplacementMarker(125,()->{
                    drive.dropdown.setPosition(.36);
                    drive.intake.setPower(1);
                })
                //retract drop down and turn off intake
                .addDisplacementMarker(145,()->{
                    drive.dropdown.setPosition(.7);
                })
                .build();

        TrajectorySequence Park = drive.trajectorySequenceBuilder(comeBackToStack.end())
                .lineToLinearHeading(new Pose2d(52, -24, Math.toRadians(180.00)))
                .addDisplacementMarker(2,()->{
                    retractSlides(drive,1);
                })
                .build();

        drive.followTrajectorySequence(redRightProp);
            drive.resetPurplePixel();
            drive.intake.setPower(0);
            drive.intake.setPower(0);
            extendSlides(drive, 1, 200);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlides(drive, 1, 300);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(0);
        drive.followTrajectorySequence(comeBackToStack);
            drive.dropdown.setPosition(.7);
            drive.intake.setPower(0);
            drive.intake.setPower(0);
            extendSlides(drive, 1, 250);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlides(drive, 1, 350);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(0);
        drive.followTrajectorySequence(Park);
    }

    public void RedCenterProp(Drive drive, int slowerVelocity, Pose2d start) {
        ElapsedTime timer = new ElapsedTime();
        drive.setPoseEstimate(start);
        TrajectorySequence RedCenterProp = drive.trajectorySequenceBuilder(start)
                .lineToLinearHeading(new Pose2d(-42, -40, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(-42, -27, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(-51, -15.5, Math.toRadians(180.00)))
                //back away from pixel stack
                .lineToSplineHeading(new Pose2d(-49, -15.5, Math.toRadians(180.00)))
                //pull towards stack
                .lineToSplineHeading(new Pose2d(-53, -15.5, Math.toRadians(180.00)))
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -15.5, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(58, -47, Math.toRadians(180.00)))


                //release purple
                .addDisplacementMarker(35.25,() -> {
                                    drive.releasePurplePixel();
                })
                //deploy dropdown
                .addDisplacementMarker(51.75,() -> {
                                    timer.reset();
                                    timer.startTime();
                                    drive.dropdown.setPosition(.399);
                                    drive.intake.setPower(.5);
                                    while(timer.time() < 1){
                                        drive.setMotorPowers(0,0,0,0);
                                    }
                })
                //retract dropdown / intake full speed
                .addDisplacementMarker(55,() -> {
                                    drive.dropdown.setPosition(1);
                                    drive.intake.setPower(1);
                })

                .build();

        TrajectorySequence comeBackToStack = drive.trajectorySequenceBuilder(RedCenterProp.end())
                //to center gate path
                .lineToLinearHeading(new Pose2d(30.84, -15.5, Math.toRadians(180.00)))
                //to pixel stack
                .lineTo(new Vector2d(-51.5, -17))
                //back away from stack
                .lineTo(new Vector2d(-49,-17))
                //drive towards stack
                .lineTo(new Vector2d(-53, -17))
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -15.5, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(56, -41, Math.toRadians(180.00)))

                .addDisplacementMarker(10,() -> {
                    retractSlides(drive,1);
                })
                .addDisplacementMarker(60,() ->{
                    stopResetSlides(drive);
                })
                //deploy dropdown
                .addDisplacementMarker(122,() -> {
                    timer.reset();
                    timer.startTime();
                    drive.dropdown.setPosition(.385);
                    drive.intake.setPower(.5);
                    while(timer.time() < 1){
                        drive.setMotorPowers(0,0,0,0);
                    }
                })
                //retract dropdown / intake full speed
                .addDisplacementMarker(125,() -> {
                    drive.dropdown.setPosition(1);
                    drive.intake.setPower(1);
                })
                //turn off intake
                .addDisplacementMarker(228,() -> {
                    drive.intake.setPower(0);
                })



                        .build();
                TrajectorySequence Park = drive.trajectorySequenceBuilder(comeBackToStack.end())
                        .lineToLinearHeading(new Pose2d(52, -20, Math.toRadians(180.00)))
                        .addDisplacementMarker(2,()->{
                            retractSlides(drive,1);
                        })
                        .build();

        drive.followTrajectorySequence(RedCenterProp);
            drive.resetPurplePixel();
            drive.intake.setPower(0);
            extendSlides(drive, 1, 200);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlides(drive, 1, 300);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(0);
        drive.followTrajectorySequence(comeBackToStack);
            drive.dropdown.setPosition(.7);
            drive.intake.setPower(0);
            extendSlides(drive, 1, 250);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlides(drive, 1, 350);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(0);
        drive.followTrajectorySequence(Park);

    }
    public void RedLeftProp(Drive drive, int slowerVelocity, Pose2d start) {
        ElapsedTime timer = new ElapsedTime();
        drive.setPoseEstimate(start);
        TrajectorySequence RedLeftProp = drive.trajectorySequenceBuilder(start)
                //spin away from wall
                .lineToLinearHeading(new Pose2d(-49, -54, Math.toRadians(125.00)))
                //crab walk past spike mark and drop off purple pixel
                .lineToLinearHeading(new Pose2d(-49, -22, Math.toRadians(125.00)))
                //align with pixel stack
                .lineToLinearHeading(new Pose2d(-51.5, -17, Math.toRadians(180.00)))
                //back away from stack
                .lineToLinearHeading(new Pose2d(-49, -17, Math.toRadians(180.00)))
                //pull towards stack
                .lineToLinearHeading(new Pose2d(-53, -17, Math.toRadians(180.00)))
                //to the backstage
                .lineToLinearHeading(new Pose2d(36, -17, Math.toRadians(180.00)))
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -17, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(58, -40, Math.toRadians(180.00)))

                //release purple
                .addDisplacementMarker(38,() -> {
                                    drive.releasePurplePixel();
                })
                //deploy dropdown
                .addDisplacementMarker(54.5,() -> {
                                    timer.reset();
                                    timer.startTime();
                                    drive.dropdown.setPosition(.399);
                                    drive.intake.setPower(.5);
                                    while(timer.time() < 1){
                                        drive.setMotorPowers(0,0,0,0);
                                    }
                })
                //retract dropdown / intake full speed
                .addDisplacementMarker(56,() -> {
                                    drive.dropdown.setPosition(1);
                                    drive.intake.setPower(1);
                })
                .build();

        TrajectorySequence comeBackToStack = drive.trajectorySequenceBuilder(RedLeftProp.end())
                //to center gate path
                .lineToLinearHeading(new Pose2d(30.84, -17, Math.toRadians(180.00)))
                //to pixel stack
                .lineTo(new Vector2d(-52, -18))
                //back away from stack
                .lineTo(new Vector2d(-49,-18))
                //drive towards stack
                .lineTo(new Vector2d(-53, -18))
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -17, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(56, -48, Math.toRadians(180.00)))

                .addDisplacementMarker(6,() -> {
                    retractSlides(drive, .5);
                })
                .addDisplacementMarker(50,() -> {
                    stopResetSlides(drive);
                })
                //deploy dropdown
                .addDisplacementMarker(118.5,()->{
                    timer.reset();
                    timer.startTime();
                    drive.dropdown.setPosition(.385);
                    drive.intake.setPower(.5);
                    while(timer.time() < 1){
                        drive.setMotorPowers(0,0,0,0);
                    }
                })
                //intake at full speed for 3rd and 4th pixel
                .addDisplacementMarker(124.5,()->{ //128.5
                    drive.dropdown.setPosition(.7);
                    drive.intake.setPower(1);
                })
                .build();
            TrajectorySequence Park = drive.trajectorySequenceBuilder(comeBackToStack.end())
                    .lineToLinearHeading(new Pose2d(54, -22, Math.toRadians(180.00)))
                    .build();

        drive.followTrajectorySequence(RedLeftProp);
            drive.resetPurplePixel();
            drive.intake.setPower(0);
            extendSlides(drive, 1, 200);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlides(drive, 1, 300);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(0);
            retractSlides(drive, 1);
        drive.followTrajectorySequence(comeBackToStack);
            drive.dropdown.setPosition(.7);
            drive.intake.setPower(0);
            extendSlides(drive, 1, 250);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlides(drive, 1, 350);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(0);
            retractSlides(drive, 1);
        drive.followTrajectorySequence(Park);
    }

    public void extendSlides(Drive drive, double speed, int position){
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
    public void stopResetSlides(Drive drive){
        drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightSlide.setPower(0);
        drive.leftSlide.setPower(0);
    }
    }
