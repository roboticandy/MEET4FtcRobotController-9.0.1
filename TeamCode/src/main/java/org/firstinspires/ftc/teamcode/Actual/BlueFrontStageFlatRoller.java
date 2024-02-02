package org.firstinspires.ftc.teamcode.Actual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Config
@Disabled
public class BlueFrontStageFlatRoller extends LinearOpMode {
    public FtcDashboard dashboard;

    public class PixelCounter implements Runnable {
        public int pixels = 1;
        boolean isPixel = false;
        boolean isIntakeOn = false;
        AnalogInput pixelSensor = hardwareMap.get(AnalogInput.class, "pixelSensor");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        @Override
        public void run() {
            while (!isStarted() || opModeIsActive()) {
                System.out.println("Voltage: " + pixelSensor.getVoltage());
                if(intake.getPower() == 0){
                    isIntakeOn = false;
                }
                if (intake.getPower() > 0 && pixelSensor.getVoltage() > .006 && !isPixel) {
                    pixels++;
                    isPixel = true;
                } else if (intake.getPower() < 0 && pixelSensor.getVoltage() > .003 && !isPixel) {
                    pixels--;
                    pixels = Range.clip(pixels,0,100);
                    isPixel = true;
                } else if (pixelSensor.getVoltage() < .009 && isPixel) {
                    isPixel = false;
                }
                if(pixels > 2){
                    System.out.println("Pixel REVERSED");
                    intake.setPower(-1);
                }else if(isIntakeOn){
                    intake.setPower(1);
                }
                System.out.println("Pixels: " + pixels);
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
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

        PixelCounter pixelCounter = new PixelCounter();
        Thread thread = new Thread(pixelCounter);
        thread.start();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        TelemetryPacket packet = new TelemetryPacket();


        //BUILD ALL BLUE TRAJECTORIES
        Pose2d startPose = new Pose2d(-35, 63, Math.toRadians(270));
        TrajectorySequence[] trajectorySequences = new TrajectorySequence[12];
        trajectorySequences = buildBlueTrajectories(drive, startPose, 15);

        while (!isStarted()) {
                if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) { // CENTER
                    drive.ledLightAdjust(31, 0, 255, 0);
                    telemetry.addLine("Detected Prop! Position:CENTER");
                    packet.addLine("Detected Prop! Position:CENTER");
                    dashboard.sendTelemetryPacket(packet);
                } else if (propDetector.centerOfProp.x >= 200) { //RIGHT
                    drive.ledLightAdjust(31, 0, 0, 255);
                    telemetry.addLine("Detected Prop! POSITION:RIGHT");
                    packet.addLine("Detected Prop! POSITION:RIGHT");
                    dashboard.sendTelemetryPacket(packet);
                } else { //LEFT
                    telemetry.addLine("Detected Prop! POSITION:LEFT");
                    drive.ledLightAdjust(31, 255, 0, 0);
                    packet.addLine("Detected Prop! POSITION:LEFT");
                    dashboard.sendTelemetryPacket(packet);
                }
            telemetry.addData("Pixels Counted: ", pixelCounter.pixels);
            telemetry.addLine(String.valueOf(drive.rightRear.getCurrentPosition()));
            telemetry.update();
        }

        if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
            BlueCenterProp(drive,trajectorySequences);
        } else if (propDetector.centerOfProp.x >= 200) { //right
            webcam.stopStreaming();
            webcam.closeCameraDevice();
            BlueRightProp(drive, trajectorySequences,pixelCounter);
        } else {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
            BlueLeftProp(drive, trajectorySequences);
        }
        stop();
        if (isStopRequested()) return;
        stop();
    }

    public void BlueLeftProp(Drive drive, TrajectorySequence[] trajectorySequences) {
        drive.followTrajectorySequence(trajectorySequences[0]);
//        pixelSequenceOne(drive);
        drive.followTrajectorySequence(trajectorySequences[1]);
        drive.resetPurplePixel();
        drive.intake.setPower(0);
        extendSlidesNew(drive, 1, 175);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(250);
        extendSlidesNew(drive, 1, 300);
        while (drive.rightSlide.isBusy()) ;
        drive.followTrajectorySequence(trajectorySequences[2]);
        pixelSequenceTwo(drive);
        drive.followTrajectorySequence(trajectorySequences[3]);
        drive.intake.setPower(0);
        drive.dropdown.setPosition(.64);
        extendSlidesNew(drive, 1, 350);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(250);
        extendSlidesNew(drive, 1, 400);
        while (drive.rightSlide.isBusy()) ;
        straight(drive, .25, 2);
    }

    public void BlueCenterProp(Drive drive, TrajectorySequence[] trajectorySequences) {
        drive.followTrajectorySequence(trajectorySequences[4]);
//        pixelSequenceOne(drive);
        drive.followTrajectorySequence(trajectorySequences[5]);
        drive.resetPurplePixel();
        drive.intake.setPower(0);
        extendSlidesNew(drive, 1, 175);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(250);
        extendSlidesNew(drive, 1, 300);
        while (drive.rightSlide.isBusy()) ;
        drive.followTrajectorySequence(trajectorySequences[6]);
        pixelSequenceTwo(drive);
        drive.followTrajectorySequence(trajectorySequences[7]);
        drive.intake.setPower(0);
        drive.dropdown.setPosition(.64);
        extendSlidesNew(drive, 1, 350);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(250);
        extendSlidesNew(drive, 1, 400);
        while (drive.rightSlide.isBusy()) ;
        straight(drive, .25, 2);
    }

    public void BlueRightProp(Drive drive,TrajectorySequence[] trajectorySequences,PixelCounter pixelCounter) {
        //EXECUTE SEQUENCE
        drive.followTrajectorySequence(trajectorySequences[8]);
        pixelSequenceOne(drive,pixelCounter);
        System.out.println("Pixels: "+pixelCounter.pixels);
        drive.followTrajectorySequence(trajectorySequences[9]);
        drive.resetPurplePixel();
        drive.intake.setPower(0);
//        extendSlidesNew(drive, 1, 175);
        extendSlidesNew(drive, 1, 250);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        System.out.println("Reset pixel count to 0");
        pixelCounter.pixels = 0;
        sleep(250);
        extendSlidesNew(drive, 1, 300);
        while (drive.rightSlide.isBusy()) ;
        drive.followTrajectorySequence(trajectorySequences[10]);
        pixelSequenceTwo(drive);
        drive.followTrajectorySequence(trajectorySequences[11]);
        drive.intake.setPower(0);
        drive.dropdown.setPosition(.64);
        extendSlidesNew(drive, 1, 350);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(250);
        extendSlidesNew(drive, 1, 400);
        while (drive.rightSlide.isBusy()) ;
        straight(drive, .25, 2);
    }

    public TrajectorySequence[] buildBlueTrajectories(Drive drive, Pose2d start, double slowerVelocity){
        TrajectorySequence[] returnTrajectories = new TrajectorySequence[12];
        drive.setPoseEstimate(start);
        //*****************************************************************************************************************************
        //BLUE LEFT
        //*****************************************************************************************************************************
        TrajectorySequence BLblueLeftProp = drive.trajectorySequenceBuilder(start)
                //pull away turn
                .lineToLinearHeading(new Pose2d(-44, 30, Math.toRadians(180)))
                //backup to spike
                .lineTo(new Vector2d(-32, 34))
                //crab walk to stack
                .lineTo(new Vector2d(-58.5, 5.5))
                // slowly hit wall with dropdown
                .lineToLinearHeading(new Pose2d(-61.5, 5.5, Math.toRadians(180)),
                        drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //release purple
                .addDisplacementMarker(39, () -> {
                    drive.releasePurplePixel();
                    drive.dropdown.setPosition(.39);

                })
                .build();
        TrajectorySequence BLtoBackDropL = drive.trajectorySequenceBuilder(BLblueLeftProp.end())
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, 6.5, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(48, 38, Math.toRadians(180.00)))

                .build();

        TrajectorySequence BLcomeBackToStack = drive.trajectorySequenceBuilder(BLtoBackDropL.end())
                //to center gate path
                .lineToLinearHeading(new Pose2d(30.84, 6.5, Math.toRadians(180.00)))
                //to pixel stack
                .lineTo(new Vector2d(-62, 6.5))
                .addDisplacementMarker(10, () -> {
                    retractSlides(drive, .5);
                    drive.dropdown.setPosition(.385);
                })
                .addDisplacementMarker(75, () -> {
                    stopResetSlides(drive);
                })
                .build();

        TrajectorySequence BLtoBackDropR = drive.trajectorySequenceBuilder(BLcomeBackToStack.end())
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, 3, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(46, 30, Math.toRadians(180.00)))
                .build();
        //*****************************************************************************************************************************
        //BLUE CENTER
        //*****************************************************************************************************************************
        TrajectorySequence BCblueCenterProp = drive.trajectorySequenceBuilder(start)
                //drop off purple pixel
                .lineToSplineHeading(new Pose2d(-53, 12, Math.toRadians(190.00)))
                //align with pixel stack
                .lineToSplineHeading(new Pose2d(-56, 6, Math.toRadians(180.00)))
                //slowly hit wall with dropdown
                .lineToLinearHeading(new Pose2d(-61, 6, Math.toRadians(180)),
                        drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //release purple && put down dropdown as a measuring device
                .addDisplacementMarker(44, () -> {
                    drive.releasePurplePixel();
                    drive.dropdown.setPosition(.385);
                })
                .build();

        TrajectorySequence BCtoBackDropC = drive.trajectorySequenceBuilder(BCblueCenterProp.end())
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, 6, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(50, 32, Math.toRadians(180.00)))
                .build();

        TrajectorySequence BCcomeBackToStack = drive.trajectorySequenceBuilder(BCtoBackDropC.end())
                //to center gate path
                .lineToLinearHeading(new Pose2d(30.84, 6, Math.toRadians(180.00)))
                //to pixel stack
                .lineTo(new Vector2d(-56, 7))
                .lineToLinearHeading(new Pose2d(-60.5, 7, Math.toRadians(180)),
                        drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(10, () -> {
                    retractSlides(drive, .5);
                    drive.dropdown.setPosition(.385);
                })
                .addDisplacementMarker(75, () -> {
                    stopResetSlides(drive);
                })
                .build();

        TrajectorySequence BCtoBackDropR = drive.trajectorySequenceBuilder(BCcomeBackToStack.end())
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, 3, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(47, 29, Math.toRadians(180.00)))
                .build();

        //*****************************************************************************************************************************
        //BLUE RIGHT
        //*****************************************************************************************************************************
        TrajectorySequence BRblueRightProp = drive.trajectorySequenceBuilder(start)
                //TEST TRAJ
                .lineToLinearHeading(new Pose2d(-34.00, 26.00, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(-48.00, 26.00, Math.toRadians(270.00)))
                .lineToLinearHeading(new Pose2d(-48.00, 20.00, Math.toRadians(270.00)))
                .splineToLinearHeading(new Pose2d(-61, 6.5, Math.toRadians(180.00)), Math.toRadians(207.16))
                .addDisplacementMarker(46, () -> {
                    drive.releasePurplePixel();
                })
                .addDisplacementMarker(60, () -> {
                    drive.dropdown.setPosition(.39);
                })

//                //spin from wall
//                .lineToLinearHeading(new Pose2d(-50, 43.98, Math.toRadians(235)))
//                //crab walk past right spike
//                .lineToLinearHeading(new Pose2d(-50, 10, Math.toRadians(235)))
//                //align with pixel stack
//                .lineToLinearHeading(new Pose2d(-58.5, 6.5, Math.toRadians(180.00)))
//                //slowly hit wall with dropdown
//                .lineToLinearHeading(new Pose2d(-61.5, 6.5, Math.toRadians(180)),
//                        drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                //release purple
//                .addDisplacementMarker(40, () -> {
//                    drive.releasePurplePixel();
//                })
//                .addDisplacementMarker(71, () -> {
//                    drive.dropdown.setPosition(.39);
//                })
                .build();
        TrajectorySequence BRtoBackDropR = drive.trajectorySequenceBuilder(BRblueRightProp.end())
                // to the backstage
                .lineToLinearHeading(new Pose2d(30.84, 6.5, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(50, 26, Math.toRadians(180.00)))
                .build();

        TrajectorySequence BRcomeBackToStack = drive.trajectorySequenceBuilder(BRtoBackDropR.end())
                //to center gate path
                .lineToLinearHeading(new Pose2d(30.84, 6, Math.toRadians(180.00)))
                //to pixel stack
                .lineTo(new Vector2d(-62, 7))
                //hit wall at slow speed
//                .lineToLinearHeading(new Pose2d(-63, 7, Math.toRadians(180)),
//                        drive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(10, () -> {
                    retractSlides(drive, .5);
                    drive.dropdown.setPosition(.385);
                })
                .addDisplacementMarker(75, () -> {
                        stopResetSlides(drive);
                })

                .build();

        TrajectorySequence BRtoBackDropC = drive.trajectorySequenceBuilder(BRcomeBackToStack.end())
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, 3, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(48, 34, Math.toRadians(180.00)))
                .build();
        //*****************************************************************************************************************************
        //LEFT
        returnTrajectories[0] =  BLblueLeftProp;
        returnTrajectories[1] =  BLtoBackDropL;
        returnTrajectories[2] =  BLcomeBackToStack;
        returnTrajectories[3] =  BLtoBackDropR;
        //CENTER
        returnTrajectories[4] =  BCblueCenterProp;
        returnTrajectories[5] =  BCtoBackDropC;
        returnTrajectories[6] =  BCcomeBackToStack;
        returnTrajectories[7] =  BCtoBackDropR;
        //RIGHT
        returnTrajectories[8] =  BRblueRightProp;
        returnTrajectories[9] =  BRtoBackDropR;
        returnTrajectories[10] = BRcomeBackToStack;
        returnTrajectories[11] = BRtoBackDropC;

        return returnTrajectories;
    }
    public void extendSlidesNew(Drive drive, double speed, int position) {
        drive.rightSlide.setTargetPosition(position);
        drive.leftSlide.setTargetPosition(position);

        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.rightSlide.setPower(speed);
        drive.leftSlide.setPower(speed);
    }

    public void pixelSequenceOne(Drive drive, PixelCounter pixelCounter) {
        straight(drive, -.25, -.125);
        drive.dropdown.setPosition(.37);
        System.out.println("Pixel pickup about to begin");
        sleep(50);
        drive.intake.setPower(1);
        straight(drive, -1, -3);
        drive.dropdown.setPosition(.6);
        sleep(50);
        straight(drive, 1, 3.5);
        if(pixelCounter.pixels > 1){
            System.out.println("Too many pixels!!");
        }
    }

    public void pixelSequenceTwo(Drive drive) {
        straight(drive, -.25, -.125);
        drive.dropdown.setPosition(.355);
        sleep(50);
        drive.intake.setPower(1);
        straight(drive, -1, -3);
        sleep(50);
        straight(drive, 1, 3);
        drive.dropdown.setPosition(.335); //.335
        sleep(50);
        drive.intake.setPower(1);
        straight(drive, -1, -3);
        sleep(50);
        straight(drive, 1, 2.5);
    }

    public void straight(Drive drive, double speed, double distance) {
        double currentParallelCount = drive.rightRear.getCurrentPosition();
        double countToAdd = distance * 336.7;
        if (distance < 0) {
            while (drive.rightRear.getCurrentPosition() < currentParallelCount - countToAdd && opModeIsActive()) {
                drive.leftFront.setPower(speed);
                drive.leftRear.setPower(speed);
                drive.rightFront.setPower(speed);
                drive.rightRear.setPower(speed);
            }
        } else {
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
        sleep(250);
    }

    public void extendSlides(Drive drive, double speed, int position) {
//        Drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.rightSlide.setTargetPosition(position);

        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.rightSlide.setPower(speed);
        while ((drive.rightSlide.getCurrentPosition() < position || drive.rightSlide.isBusy()) && opModeIsActive()) {
            drive.leftSlide.setPower(drive.rightSlide.getPower());
            telemetry.addLine(String.valueOf(drive.rightSlide.getPower()));
            telemetry.addLine(String.valueOf(drive.leftSlide.getPower()));
            telemetry.update();
        }
        drive.leftSlide.setPower(0);
    }

    public void extendSlidesPullAway(Drive drive, double speed, int position) {
        drive.rightSlide.setTargetPosition(position);

        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.rightSlide.setPower(speed);
        while (drive.rightSlide.isBusy()) {
            drive.leftSlide.setPower(drive.rightSlide.getPower());
            telemetry.addLine(String.valueOf(drive.rightSlide.getPower()));
            telemetry.addLine(String.valueOf(drive.leftSlide.getPower()));
            telemetry.update();
        }
        drive.leftSlide.setPower(0);
    }

    public void retractSlides(Drive drive, double speed) {
        drive.pixelScore.setPosition(.46);
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
