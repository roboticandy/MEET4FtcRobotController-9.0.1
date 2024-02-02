package org.firstinspires.ftc.teamcode.Actual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Config
public class RedFrontStageFlatRoller extends LinearOpMode {
    public FtcDashboard dashboard;
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
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        TelemetryPacket packet = new TelemetryPacket();

        Pose2d startPose = new Pose2d(-35, -63, Math.toRadians(90));
        TrajectorySequence[] trajectorySequences = new TrajectorySequence[12];
        trajectorySequences = buildBlueTrajectories(drive, startPose, 15);
        while (!isStarted()) {
                if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) { // CENTER
                    drive.ledLightAdjust(31,0,255,0);
                    telemetry.addLine("Detected Prop! Position:CENTER");
                    packet.addLine("Detected Prop! Position:CENTER");
                    telemetry.update();
                    dashboard.sendTelemetryPacket(packet);
                } else if (propDetector.centerOfProp.x >= 200) {
                    drive.ledLightAdjust(31,0,0,255);
                    telemetry.addLine("Detected Prop! POSITION:RIGHT");
                    packet.addLine("Detected Prop! POSITION:RIGHT");
                    telemetry.update();
                    dashboard.sendTelemetryPacket(packet);
                } else { //LEFT
                    drive.ledLightAdjust(31,255,0,0);
                    telemetry.addLine("Detected Prop! POSITION:LEFT");
                    packet.addLine("Detected Prop! POSITION:LEFT");
                    telemetry.update();
                    dashboard.sendTelemetryPacket(packet);
                }
        }

        if (propDetector.centerOfProp.x > 0 && propDetector.centerOfProp.x < 200) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
            RedCenterProp(drive,trajectorySequences);
        } else if (propDetector.centerOfProp.x >= 200) { //right
            webcam.stopStreaming();
            webcam.closeCameraDevice();
            RedRightProp(drive, trajectorySequences);
        } else {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
            RedLeftProp(drive, trajectorySequences);
        }
        stop();
        if (isStopRequested()) return;
        stop();
    }

    public void RedRightProp(Drive drive, TrajectorySequence[] trajectorySequences) {
        //EXECUTE SEQUENCE
        drive.followTrajectorySequence(trajectorySequences[8]);
        pixelSequenceOne(drive);
        drive.followTrajectorySequence(trajectorySequences[9]);
        drive.resetPurplePixel();
        drive.intake.setPower(0);
        extendSlidesNew(drive, 1, 225);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(500);
        extendSlidesNew(drive, 1, 300);
        while (drive.rightSlide.isBusy()) ;
        drive.followTrajectorySequence(trajectorySequences[10]);
        pixelSequenceTwo(drive);
        drive.followTrajectorySequence(trajectorySequences[11]);
        drive.intake.setPower(0);
        extendSlidesNew(drive, 1, 350);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(500);
        extendSlidesNew(drive, 1, 400);
        while (drive.rightSlide.isBusy()) ;
        straight(drive, .25, 2);
    }

    public void RedCenterProp(Drive drive, TrajectorySequence[] trajectorySequences) {
        drive.followTrajectorySequence(trajectorySequences[4]);
        pixelSequenceOne(drive);
        drive.followTrajectorySequence(trajectorySequences[5]);
        drive.resetPurplePixel();
        drive.intake.setPower(0);
        extendSlidesNew(drive, 1, 225);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(500);
        extendSlidesNew(drive, 1, 300);
        while (drive.rightSlide.isBusy()) ;
        drive.followTrajectorySequence(trajectorySequences[6]);
        pixelSequenceTwo(drive);
        drive.followTrajectorySequence(trajectorySequences[7]);
        drive.intake.setPower(0);
        extendSlidesNew(drive, 1, 350);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(500);
        extendSlidesNew(drive, 1, 400);
        while (drive.rightSlide.isBusy()) ;
        straight(drive, .25, 2);
    }
    public void RedLeftProp(Drive drive,TrajectorySequence[] trajectorySequences) {
        //EXECUTE SEQUENCE
        drive.followTrajectorySequence(trajectorySequences[0]);
        pixelSequenceOne(drive);
        drive.followTrajectorySequence(trajectorySequences[1]);
        drive.resetPurplePixel();
        drive.intake.setPower(0);
        extendSlidesNew(drive, 1, 225);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(500);
        extendSlidesNew(drive, 1, 300);
        while (drive.rightSlide.isBusy()) ;
        drive.followTrajectorySequence(trajectorySequences[2]);
        pixelSequenceTwo(drive);
        drive.followTrajectorySequence(trajectorySequences[3]);
        drive.intake.setPower(0);
        extendSlidesNew(drive, 1, 350);
        while (drive.rightSlide.isBusy()) ;
        drive.pixelScore.setPosition(.559);
        sleep(500);
        extendSlidesNew(drive, 1, 400);
        while (drive.rightSlide.isBusy()) ;
        straight(drive, .25, 2);
    }

    public TrajectorySequence[] buildBlueTrajectories(Drive drive, Pose2d start, double slowerVelocity) {
        TrajectorySequence[] returnTrajectories = new TrajectorySequence[12];
        drive.setPoseEstimate(start);
        //*****************************************************************************************************************************
        //RED RIGHT
        //*****************************************************************************************************************************
        TrajectorySequence RRredRightProp = drive.trajectorySequenceBuilder(start)
                //pull away turn
                .lineToLinearHeading(new Pose2d(-40, -41, Math.toRadians(180)))
                //backup to spike
                .lineTo(new Vector2d(-25, -41))
                //crab walk to stack
                .lineTo(new Vector2d(-46.5, -14.75))
                // hit wall
                .lineTo(new Vector2d(-54.5, -16.75))
//                release purple
                .addDisplacementMarker(39,() -> {
                    drive.releasePurplePixel();
                    drive.dropdown.setPosition(.385);
                })
                .build();
        TrajectorySequence RRtoBackdropR = drive.trajectorySequenceBuilder(RRredRightProp.end())
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -16.75, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(57, -48, Math.toRadians(180.00)))
                .build();
        TrajectorySequence RRcomeBackToStack = drive.trajectorySequenceBuilder(RRtoBackdropR.end())
                //to center gate path
                .lineToLinearHeading(new Pose2d(30.84, -16.75, Math.toRadians(180.00)))
                //to pixel stack
                .lineTo(new Vector2d(-53, -16.75))


                // retract slides
                .addDisplacementMarker(6,() -> {
                    retractSlides(drive, .5);
                    drive.dropdown.setPosition(.385);
                })
                .addDisplacementMarker(75, () -> {
                    stopResetSlides(drive);
                })
//                .addDisplacementMarker(50,() -> {
//                    stopResetSlides(drive);
//                })
                .build();
        TrajectorySequence RRtoBackdropC = drive.trajectorySequenceBuilder(RRtoBackdropR.end())
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -15.75, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(56, -40, Math.toRadians(180.00)))

                .build();

        TrajectorySequence RRPark = drive.trajectorySequenceBuilder(RRtoBackdropR.end())
                .lineToLinearHeading(new Pose2d(52, -24, Math.toRadians(180.00)))
                .addDisplacementMarker(2,()->{
                    retractSlides(drive,5);
                })
                .build();
//*****************************************************************************************************************************
        //RED CENTER
        //*****************************************************************************************************************************
        drive.setPoseEstimate(start);
        TrajectorySequence RCredCenterProp = drive.trajectorySequenceBuilder(start)
                // spin away from wall
                .lineToLinearHeading(new Pose2d(-42, -40, Math.toRadians(180.00)))
                // towards pixel stack
                .lineToSplineHeading(new Pose2d(-42, -27, Math.toRadians(180.00)))
                // allign with pixel stack
                .lineToSplineHeading(new Pose2d(-45, -16.5, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(-54, -16.5, Math.toRadians(180.00)))

                //release purple
                .addDisplacementMarker(35.25,() -> {
                    drive.releasePurplePixel();
                    drive.dropdown.setPosition(.385);
                })

                .build();
        TrajectorySequence RCtoBackDropC = drive.trajectorySequenceBuilder(RCredCenterProp.end())
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -15.5, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(58, -45, Math.toRadians(180.00)))
                .build();
        TrajectorySequence RCcomeBackToStack = drive.trajectorySequenceBuilder(RCtoBackDropC.end())
                //to center gate path
                .lineToLinearHeading(new Pose2d(30.84, -15.5, Math.toRadians(180.00)))
                //to pixel stack
                .lineTo(new Vector2d(-54.5, -17))

                .addDisplacementMarker(10,() -> {
                    retractSlides(drive,.5);
                    drive.dropdown.setPosition(.385);

                })
                .addDisplacementMarker(75, () -> {
                    stopResetSlides(drive);
                })
//                .addDisplacementMarker(60,() ->{
//                    stopResetSlides(drive);
//                })



                .build();
        TrajectorySequence RCtoBackDropR = drive.trajectorySequenceBuilder(RCcomeBackToStack.end())
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -15.5, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(56, -38, Math.toRadians(180.00)))
                .build();
        TrajectorySequence RCPark = drive.trajectorySequenceBuilder(RCcomeBackToStack.end())
                .lineToLinearHeading(new Pose2d(52, -20, Math.toRadians(180.00)))
                .addDisplacementMarker(2,()->{
                    retractSlides(drive,.5);
                })
                .build();
        drive.setPoseEstimate(start);
        //*****************************************************************************************************************************
        //RED LEFT
        //*****************************************************************************************************************************
        TrajectorySequence RLredLeftProp = drive.trajectorySequenceBuilder(start)
                //spin away from wall
                .lineToLinearHeading(new Pose2d(-49, -54, Math.toRadians(125.00)))
                //crab walk past spike mark and drop off purple pixel
                .lineToLinearHeading(new Pose2d(-49, -20, Math.toRadians(125.00)))
                //align with pixel stack
                .lineToLinearHeading(new Pose2d(-45.5, -17, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(-53.5 /* 54.5 */, -17, Math.toRadians(180.00)))


                //release purple
                .addDisplacementMarker(38,() -> {
                    drive.releasePurplePixel();
                    drive.dropdown.setPosition(.385);
                })
                .build();
        TrajectorySequence RLtoBackDropL = drive.trajectorySequenceBuilder(RLredLeftProp.end())
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -17, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(58, -37, Math.toRadians(180.00)))
                .build();
        TrajectorySequence RLcomeBackToStack = drive.trajectorySequenceBuilder(RLtoBackDropL.end())
                //to center gate path
                .lineToLinearHeading(new Pose2d(30.84, -17, Math.toRadians(180.00)))
                //to pixel stack
                .lineTo(new Vector2d(-57, -18))

                .addDisplacementMarker(10,() -> {
                    retractSlides(drive, .5);
                    drive.dropdown.setPosition(.385);
                })
                .addDisplacementMarker(75, () -> {
                    stopResetSlides(drive);
                })
                .build();
        TrajectorySequence RLtoBackDropC = drive.trajectorySequenceBuilder(RLcomeBackToStack.end())
                //to the backstage
                .lineToLinearHeading(new Pose2d(30.84, -17, Math.toRadians(180.00)))
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(56, -47, Math.toRadians(180.00)))
                .build();
        TrajectorySequence RLPark = drive.trajectorySequenceBuilder(RLtoBackDropC.end())
                .lineToLinearHeading(new Pose2d(54, -22, Math.toRadians(180.00)))
                .build();
        //*****************************************************************************************************************************
        //LEFT
        returnTrajectories[0] =  RLredLeftProp;
        returnTrajectories[1] =  RLtoBackDropL;
        returnTrajectories[2] =  RLcomeBackToStack;
        returnTrajectories[3] =  RLtoBackDropC;
        //CENTER
        returnTrajectories[4] =  RCredCenterProp;
        returnTrajectories[5] =  RCtoBackDropC;
        returnTrajectories[6] =  RCcomeBackToStack;
        returnTrajectories[7] =  RCtoBackDropR;
        //RIGHT
        returnTrajectories[8] =   RRredRightProp;
        returnTrajectories[9] =   RRtoBackdropR;
        returnTrajectories[10] =  RRcomeBackToStack;
        returnTrajectories[11] =  RRtoBackdropC;

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

    public void pixelSequenceOne(Drive drive) {
        straight(drive, -.25, -.125);
        drive.dropdown.setPosition(.37);
        sleep(250);
        drive.intake.setPower(1);
        straight(drive, -.5, -3);
        drive.dropdown.setPosition(.6);
        sleep(100);
        straight(drive, .5, 3.5);
    }

    public void pixelSequenceTwo(Drive drive) {
        straight(drive, -.25, -.125);
        drive.dropdown.setPosition(.355);
        sleep(250);
        drive.intake.setPower(1);
        straight(drive, -.5, -3);
        sleep(100);
        straight(drive, .5, 4);
        drive.dropdown.setPosition(.34);
        sleep(250);
        drive.intake.setPower(1);
        straight(drive, -.5, -3);
        sleep(100);
        straight(drive, .5, 2.5);
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
        sleep(100);
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