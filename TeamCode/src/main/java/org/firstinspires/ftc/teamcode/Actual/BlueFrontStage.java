package org.firstinspires.ftc.teamcode.Actual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
public class BlueFrontStage extends LinearOpMode {
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        OpenCvWebcam webcam;
        Drive drive = new Drive(hardwareMap);
        TrajectorySequence[] currentTrajSeq = new TrajectorySequence[4];
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
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        TelemetryPacket packet = new TelemetryPacket();

        Vector2d leftSpike = new Vector2d(1, 1);
        Vector2d rightSpike = new Vector2d(1, 1);
        Vector2d centerSpike = new Vector2d(1, 1);
        Vector2d leftBackdrop = new Vector2d(1, 1);
        Vector2d rightBackdrop = new Vector2d(1, 1);
        Vector2d centerBackdrop = new Vector2d(1, 1);
        Pose2d startPose = new Pose2d(-35, 63, Math.toRadians(270));
        while (!isStarted()) {

            {
                if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) { // CENTER
                    drive.ledLightAdjust(31,0,255,0);
                    telemetry.addLine("Detected Prop! Position:CENTER");
                    packet.addLine("Detected Prop! Position:CENTER");
                    telemetry.update();
                    dashboard.sendTelemetryPacket(packet);
                } else if (propDetector.centerOfProp.x >= 200) { //RIGHT
                    drive.ledLightAdjust(31,0,0,255);
                    telemetry.addLine("Detected Prop! POSITION:RIGHT");
                    packet.addLine("Detected Prop! POSITION:RIGHT");
                    telemetry.update();
                    dashboard.sendTelemetryPacket(packet);
                } else { //LEFT
                    telemetry.addLine("Detected Prop! POSITION:LEFT");
                    drive.ledLightAdjust(31,255,0,0);
                    packet.addLine("Detected Prop! POSITION:LEFT");
                    telemetry.update();
                    dashboard.sendTelemetryPacket(packet);
                }
            }
        }

        if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) {
            BlueCenterProp(drive,35,startPose);
        } else if (propDetector.centerOfProp.x >= 200) { //right
            // (propPos == 3){
            BlueRightProp(drive, 30, startPose);
        } else {
            BlueLeftProp(drive,35,startPose);
        }
        stop();
        if (isStopRequested()) return;
        stop();
    }

    public void BlueLeftProp(Drive drive, int slowerVelocity, Pose2d start) {
        ElapsedTime timer = new ElapsedTime();
        drive.setPoseEstimate(start);
        drive.setMotorPowers(0,0,0,0);
        TrajectorySequence blueLeftProp = drive.trajectorySequenceBuilder(start)
                //pull away turn
                .lineToLinearHeading(new Pose2d(-44, 30, Math.toRadians(180)))
                //backup to spike
                .lineTo(new Vector2d(-32, 34))
                //crab walk to stack
                .lineTo(new Vector2d(-58.5, 5.5))
                //back away from stack
                .lineTo(new Vector2d(-57,5.5))
                //drive towards stack
                .lineTo(new Vector2d(-61.25, 5.5))
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, 6.5, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(48, 41, Math.toRadians(180.00)))


                //release purple
                .addDisplacementMarker(39,() -> {
                    drive.releasePurplePixel();
                })
                //deploy dropdown
                .addDisplacementMarker(83,() -> {
                    timer.reset();
                    timer.startTime();
                    drive.dropdown.setPosition(.399);
                    drive.intake.setPower(.5);
                    while(timer.time() < 1){
                        drive.setMotorPowers(0,0,0,0);
                    }
                })
                //retract dropdown / intake full speed
                .addDisplacementMarker(84,() -> {
                    drive.dropdown.setPosition(1);
                    drive.intake.setPower(1);
                })
                //turn off intake
                .addDisplacementMarker(133,() -> {
                    drive.intake.setPower(0);
                })
//                //extend slides
//                .addDisplacementMarker(180,() -> {
//                    extendSlidesNew(drive, .4,250);
//                })
//                //score pixels
//                .addDisplacementMarker(216.5,() -> {
//                    drive.pixelScore.setPosition(1);
//                    timer.reset();
//                    timer.startTime();
//                    while(timer.time() < 1.5){
//                        drive.setMotorPowers(0,0,0,0);
//                    }
//                })
//                //close pixel door and retract slides
//                .addDisplacementMarker(220.5,() -> {
//                    drive.pixelScore.setPosition(0);
//                    retractSlides(drive, .25);
//                })

                //extend slides
//                .addDisplacementMarker(447.5,()->{
//                    extendSlidesNew(drive, .4,250);
//                })
                //score pixel 3 and 4
//                .addDisplacementMarker(491.5,()->{
//                    drive.pixelScore.setPosition(1);
//                    timer.reset();
//                    timer.startTime();
//                    while(timer.time() < 1.5){
//                    }
//                })
//                //close pixel door retract slides
//                .addDisplacementMarker(498.5,()->{
//                    drive.pixelScore.setPosition(0);
//                    retractSlides(drive,.25);
//                })
                .build();
            TrajectorySequence comeBackToStack = drive.trajectorySequenceBuilder(blueLeftProp.end())
                    //to center gate path
                    .lineToLinearHeading(new Pose2d(30.84, 6.5, Math.toRadians(180.00)))
                    //to pixel stack
                    .lineTo(new Vector2d(-62, 6.5))
                    //back away from stack
                    .lineTo(new Vector2d(-57,6))
                    //drive towards stack
                    .lineTo(new Vector2d(-61.25, 6))
                    //to the backstage
                    .lineToLinearHeading(new Pose2d(30.84, 6.5, Math.toRadians(180.00)))
                    //crab walk to backdrop
                    .lineToLinearHeading(new Pose2d(46, 35, Math.toRadians(180.00)))


                    .addDisplacementMarker(10,() -> {
                        retractSlides(drive,1);
                    })
                    .addDisplacementMarker(90,() ->{
                        stopResetSlides(drive);
                    })
                    //deploy dropdown
                    .addDisplacementMarker(130.5,()->{
                        drive.dropdown.setPosition(.385);
                        drive.intake.setPower(.5);
                    })
                    //intake at full speed for 3rd and 4th pixel
                    .addDisplacementMarker(136.5,()->{
                        drive.dropdown.setPosition(.36);
                        drive.intake.setPower(1);
                    })
                    //retract drop down and turn off intake
                    .addDisplacementMarker(236.5,()->{
                        drive.dropdown.setPosition(.7);
                        drive.intake.setPower(0);
                    })
                            .build();
            TrajectorySequence Park = drive.trajectorySequenceBuilder(comeBackToStack.end())
                    .lineToLinearHeading(new Pose2d(46, 16, Math.toRadians(180.00)))
                    .addDisplacementMarker(2,()->{
                        retractSlides(drive,1);
                    })
                    .build();

        drive.followTrajectorySequence(blueLeftProp);
            drive.resetPurplePixel();
            drive.intake.setPower(0);
            extendSlides(drive, 1, 200);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlidesNew(drive, 1, 300);
            while(drive.rightSlide.isBusy());
        drive.followTrajectorySequence(comeBackToStack);
            drive.intake.setPower(0);
            extendSlides(drive, 1, 250);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlidesNew(drive, 1, 350);
            while(drive.rightSlide.isBusy());
        drive.followTrajectorySequence(Park);
    }

    public void BlueCenterProp(Drive drive, int slowerVelocity, Pose2d start) {
        ElapsedTime timer = new ElapsedTime();
        drive.setPoseEstimate(start);
        TrajectorySequence blueCenterProp = drive.trajectorySequenceBuilder(start)
                //drop off purple pixel
                .lineToSplineHeading(new Pose2d(-53, 12, Math.toRadians(190.00)))
                //align with pixel stack
                .lineToSplineHeading(new Pose2d(-60, 6, Math.toRadians(180.00)))
                //back away from pixel stack
                .lineToSplineHeading(new Pose2d(-58, 6, Math.toRadians(180.00)))
                //pull towards stack
                .lineToSplineHeading(new Pose2d(-61, 6, Math.toRadians(180.00)))
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, 6.5, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(180.00)))

                //release purple
                .addDisplacementMarker(44,() -> {
                                    drive.releasePurplePixel();
                })
                //deploy dropdown
                .addDisplacementMarker(62.5,() -> {
                                    timer.reset();
                                    timer.startTime();
                                    drive.dropdown.setPosition(.399);
                                    drive.intake.setPower(.5);
                                    while(timer.time() < 1){
                                        drive.setMotorPowers(0,0,0,0);
                                    }
                })
                //retract dropdown / intake full speed
                .addDisplacementMarker(65,() -> {
                                    drive.dropdown.setPosition(1);
                                    drive.intake.setPower(1);
                })
                //turn off intake
                .addDisplacementMarker(132,() -> {
                                    drive.intake.setPower(0);
                })
                .build();


                TrajectorySequence comeBackToStack = drive.trajectorySequenceBuilder(blueCenterProp.end())
                //to center gate path
                .lineToLinearHeading(new Pose2d(30.84, 6.5, Math.toRadians(180.00)))
                //to pixel stack
                .lineTo(new Vector2d(-62, 6.5))
                //back away from stack
                .lineTo(new Vector2d(-57,6))
                //drive towards stack
                .lineTo(new Vector2d(-61.25, 6))
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, 6.5, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(46, 35, Math.toRadians(180.00)))


                .addDisplacementMarker(10,() -> {
                    retractSlides(drive,1);
                })
                .addDisplacementMarker(90,() ->{
                    stopResetSlides(drive);
                })
                //deploy dropdown
                .addDisplacementMarker(126.5,() -> {
                    timer.reset();
                    timer.startTime();
                    drive.dropdown.setPosition(.385);
                    drive.intake.setPower(.5);
                    while(timer.time() < 1){
                        drive.setMotorPowers(0,0,0,0);
                    }
                })
                //retract dropdown / intake full speed
                .addDisplacementMarker(128,() -> {
                    drive.dropdown.setPosition(1);
                    drive.intake.setPower(1);
                })
                //turn off intake
                .addDisplacementMarker(228,() -> {
                    drive.intake.setPower(0);
                })

//                    //extend slides
//                    .addDisplacementMarker(420,()->{
//                        extendSlidesNew(drive, .4,250);
//                    })
//                    //score pixel 3 and 4
//                    .addDisplacementMarker(449.5,()->{
//                        extendSlidesNew(drive, .4,275);
//                        while(drive.leftSlide.isBusy() && drive.rightSlide.isBusy());
//                        drive.pixelScore.setPosition(.6);
//                        timer.reset();
//                        timer.startTime();
//                        while(timer.time() < 1.5){
//                            drive.setMotorPowers(0,0,0,0);
//                        }
//                    })
//                    //close pixel door retract slides
//                    .addDisplacementMarker(470,()->{
//                        drive.pixelScore.setPosition(0);
//                        retractSlides(drive,.25);
//                    })
                            .build();
        TrajectorySequence Park = drive.trajectorySequenceBuilder(comeBackToStack.end())
                .lineToLinearHeading(new Pose2d(46, 16, Math.toRadians(180.00)))
                .addDisplacementMarker(2,()->{
                    retractSlides(drive,1);
                })
                        .build();

        drive.followTrajectorySequence(blueCenterProp);
            drive.resetPurplePixel();
            drive.intake.setPower(0);
            extendSlides(drive, 1, 200);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlidesNew(drive, 1, 300);
            while(drive.rightSlide.isBusy());
        drive.followTrajectorySequence(comeBackToStack);
            drive.intake.setPower(0);
            extendSlides(drive, 1, 350);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlidesNew(drive, 1, 400);
            while(drive.rightSlide.isBusy());
        drive.followTrajectorySequence(Park);
    }
    public void BlueRightProp(Drive drive, int slowerVelocity, Pose2d start) {
        ElapsedTime timer = new ElapsedTime();
        drive.setPoseEstimate(start);
        TrajectorySequence BlueRightProp = drive.trajectorySequenceBuilder(start)
                //spin from wall
                .lineToLinearHeading(new Pose2d(-50, 43.98, Math.toRadians(235)))
                //crab walk past right spike
                .lineToLinearHeading(new Pose2d(-50, 10, Math.toRadians(235)))
                //align with pixel stack
                .lineToLinearHeading(new Pose2d(-60.5, 6.5, Math.toRadians(180.00)))
                //back away from stack
                .lineToLinearHeading(new Pose2d(-59, 6.5, Math.toRadians(180.00)))
                //pull towards stack
                .lineToLinearHeading(new Pose2d(-62, 6.5, Math.toRadians(180.00)))
                //drop off prop
                .lineToLinearHeading(new Pose2d(36.84, 6.5, Math.toRadians(180.00)))
                //comeback
                .lineToLinearHeading(new Pose2d(30.84, 6.5, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(48, 31, Math.toRadians(180.00)))

                //release purple
                .addDisplacementMarker(40,() -> {
                    drive.releasePurplePixel();
                })
                //deploy dropdown
                .addDisplacementMarker(64,() -> {
                    timer.reset();
                    timer.startTime();
                    drive.dropdown.setPosition(.399);
                    drive.intake.setPower(.5);
                    while(timer.time() < 1){
                        drive.setMotorPowers(0,0,0,0);
                    }
                })
                //retract dropdown / intake full speed
                .addDisplacementMarker(66,() -> {
                    drive.dropdown.setPosition(.4);
                    drive.intake.setPower(1);
                })
                //turn off intake
//                .addDisplacementMarker(132,() -> {
//                    drive.intake.setPower(0);
//                })

                .build();

        TrajectorySequence comeBackToStack = drive.trajectorySequenceBuilder(BlueRightProp.end())
                //to center gate path
                .lineToLinearHeading(new Pose2d(30.84,6, Math.toRadians(180.00)))
                //to pixel stack
                .lineTo(new Vector2d(-62, 6))
                //back away from stack
                .lineTo(new Vector2d(-59,6))
                //drive towards stack
                .lineTo(new Vector2d(-62.5, 6))
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, 6, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(46, 32, Math.toRadians(180.00)))


                .addDisplacementMarker(6,() -> {
                    retractSlides(drive, 1);
                })
                .addDisplacementMarker(90,() -> {
                    stopResetSlides(drive);
                })

                .addDisplacementMarker(120,() -> {
                    timer.reset();
                    timer.startTime();
                    drive.dropdown.setPosition(.385);
                    drive.intake.setPower(.5);
                    while(timer.time() < 1){
                        drive.setMotorPowers(0,0,0,0);
                    }
                    drive.dropdown.setPosition(.33);
                })
                //retract dropdown / intake full speed
                .addDisplacementMarker(125,() -> {
                    drive.dropdown.setPosition(.4);
                    drive.intake.setPower(1);
                })
                //turn off intake
//                .addDisplacementMarker(233.5,() -> {
//                    drive.intake.setPower(0);
//                })
//                //deploy dropdown
//                .addDisplacementMarker(310,()->{
//                                    drive.dropdown.setPosition(.385);
//                                    drive.intake.setPower(.5);
//                                    resetSlideEncoder(drive);
//                })
//                //intake at full speed for 3rd and 4th pixel
//                .addDisplacementMarker(320,()->{
//                                    drive.dropdown.setPosition(.36);
//                                    drive.intake.setPower(1);
//                })
//                //retract drop down and turn off intake
//                .addDisplacementMarker(384,()->{
//                                    drive.dropdown.setPosition(.7);
//                                    drive.intake.setPower(0);
//                })
//                //extend slides
//                .addDisplacementMarker(414,()->{
//                                    extendSlidesNew(drive, .4,250);
//                })
//                //score pixel 3 and 4
//                .addDisplacementMarker(457,()->{
//                                    drive.pixelScore.setPosition(.6);
//                                    timer.reset();
//                                    timer.startTime();
//                                    while(timer.time() < 1.5){
//                                        drive.setMotorPowers(0,0,0,0);
//                                    }
//                })
//                //close pixel door retract slides
//                .addDisplacementMarker(467,()->{
//                                    drive.pixelScore.setPosition(0);
//                                    extendSlidesNew(drive,.25,0);
////                                    retractSlides(drive,.25);
//                })
                .build();

        TrajectorySequence Park = drive.trajectorySequenceBuilder(comeBackToStack.end())
                .addDisplacementMarker(2,()->{
                    retractSlides(drive,1);
                })
                .forward(4)
                .build();



        //EXECUTE SEQUENCE
        drive.followTrajectorySequence(BlueRightProp);
            drive.resetPurplePixel();
            drive.dropdown.setPosition(.7);
            drive.intake.setPower(0);
            extendSlides(drive, 1, 200);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlidesNew(drive, 1, 300);
            while(drive.rightSlide.isBusy());
        drive.followTrajectorySequence(comeBackToStack);
            drive.dropdown.setPosition(.7);
            drive.intake.setPower(0);
            extendSlides(drive, 1, 250);
            while(drive.rightSlide.isBusy());
            drive.pixelScore.setPosition(.559);
            sleep(500);
            extendSlidesNew(drive, 1, 350);
            while(drive.rightSlide.isBusy());
            retractSlides(drive, 1);
        drive.followTrajectorySequence(Park);
    }
    public void extendSlidesNew(Drive drive, double speed, int position){
        drive.rightSlide.setTargetPosition(position);
        drive.leftSlide.setTargetPosition(position);

        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.rightSlide.setPower(speed);
        drive.leftSlide.setPower(speed);
    }

    public void resetSlideEncoder(Drive drive){
        drive.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void extendSlides(Drive drive, double speed, int position){
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
        drive.pixelScore.setPosition(0);
        sleep(200);
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
