package org.firstinspires.ftc.teamcode.Actual;

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
@Disabled
public class ABackdropTest extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(30.84, 6.5, Math.toRadians(180));

        while (!isStarted()) {
        }

        Backdrop(drive, startPose);
    }

    public void Backdrop(Drive drive, Pose2d start) {
        ElapsedTime timer = new ElapsedTime();
        drive.setPoseEstimate(start);
        TrajectorySequence backdrop = drive.trajectorySequenceBuilder(start)
                //crab walk to backdrop
                .lineToLinearHeading(new Pose2d(46, 36, Math.toRadians(180.00)))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(backdrop.end())
                //Park
                .lineToLinearHeading(new Pose2d(46, 16, Math.toRadians(180.00)))
                .build();

        drive.followTrajectorySequence(backdrop);
        extendSlides(drive, 1, 200);
        while(drive.rightSlide.isBusy());
        drive.pixelScore.setPosition(.559);
        sleep(500);
        extendSlidesNew(drive, 1, 300);
        while(drive.rightSlide.isBusy());
        retractSlides(drive, .5);
        drive.followTrajectorySequence(park);
        requestOpModeStop();
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
    public void retractSlides(Drive drive, double speed){
        drive.rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.rightSlide.setPower(-speed);
        drive.leftSlide.setPower(-speed);
    }

}






