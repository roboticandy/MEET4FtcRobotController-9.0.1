package org.firstinspires.ftc.teamcode.Actual;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden Drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "Drive")
public class RamsTeleOp extends LinearOpMode {
    boolean autoRetractSlides = false;
    public class PixelCounter implements Runnable {
        public int pixels = 0;
        boolean isPixel = false;
        AnalogInput pixelSensor = hardwareMap.get(AnalogInput.class, "pixelSensor");

        @Override
        public void run() {
            while (!isStarted() || opModeIsActive()) {
                    System.out.println("Voltage: " + pixelSensor.getVoltage());
                    if (pixelSensor.getVoltage() > .006 && !isPixel) {
                        pixels++;
                        isPixel = true;
                    } else if (pixelSensor.getVoltage() < .003 && isPixel) {
                        isPixel = false;
                    }
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);

        boolean upLast = false;
        int intakeValue = -1;
        int position = 250;
        boolean upPressed = false;
        boolean downPressed = false;
        boolean upLast1 = false;
        boolean downLast = false;
        boolean yPressed = false;

//        Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.ledLightAdjust(0, 70, 0, 175);

        PixelCounter pixelCounter = new PixelCounter();
        Thread thread = new Thread(pixelCounter);
        thread.start();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_stick_y == 0 &&
                    gamepad1.left_stick_x == 0 &&
                    gamepad1.right_stick_x == 0 &&
                    gamepad1.dpad_up) {
                drive.leftFront.setPower(.50);
                drive.leftRear.setPower(.50);
                drive.rightFront.setPower(.50);
                drive.rightRear.setPower(.50);
            }else if(gamepad1.left_stick_y == 0 &&
                    gamepad1.left_stick_x == 0 &&
                    gamepad1.right_stick_x == 0 &&
                    gamepad1.dpad_down){
                drive.leftFront.setPower(-.50);
                drive.leftRear.setPower(-.50);
                drive.rightFront.setPower(-.50);
                drive.rightRear.setPower(-.50);
            }else if(gamepad1.left_stick_y == 0 &&
                    gamepad1.left_stick_x == 0 &&
                    gamepad1.right_stick_x == 0 &&
                    gamepad1.dpad_right) {
                drive.leftFront.setPower(-.60);
                drive.leftRear.setPower(.60);
                drive.rightFront.setPower(.60);
                drive.rightRear.setPower(-.60);
            }else if(gamepad1.left_stick_y == 0 &&
                    gamepad1.left_stick_x == 0 &&
                    gamepad1.right_stick_x == 0 &&
                    gamepad1.dpad_left) {
                drive.leftFront.setPower(.60);
                drive.leftRear.setPower(-.60);
                drive.rightFront.setPower(-.60);
                drive.rightRear.setPower(.60);
            }else{
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }

            if (gamepad1.left_bumper) {
                drive.dropdown.setPosition(Range.clip(drive.dropdown.getPosition() + .1,.33,.60));
            } else if (gamepad1.right_bumper) {
                drive.dropdown.setPosition(Range.clip(drive.dropdown.getPosition() - .1, .33, .60)); // .3
            }
            if(gamepad2.back){
                stopResetSlides(drive);
            }

            if (gamepad2.right_stick_y < 0 && !upLast) {
                upLast = true;
            } else if (gamepad2.right_stick_y > 0) {
                upLast = false;
            }

            if (gamepad2.y) {
                extendSlides(drive, 1, position);
                yPressed = true;
//                telemetry.addData("y Pressed ", yPressed);
            } else if(gamepad2.b) {
                retractSlides(drive, 1);
                upLast = false;
                yPressed = false;
//                telemetry.addData("B Pressed ","");
            }else if (gamepad2.right_stick_y != 0) {
                drive.rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.rightSlide.setPower(-gamepad2.right_stick_y);
                drive.leftSlide.setPower(-gamepad2.right_stick_y);
                yPressed = false;
//                telemetry.addData("Joystick Control ","");
            } else if (upLast) {
                //this increases the strength of the BRAKE
                drive.leftSlide.setPower(0.0030125);
                drive.rightSlide.setPower(0.0030125);
                yPressed = false;
//                telemetry.addData("Joystick Brake ","");
            } else if(drive.rightSlide.getCurrentPosition() < 10 && drive.leftSlide.getCurrentPosition() < 10 && yPressed == false){
                areSlidesRetractedThenReset(drive);
                telemetry.addData("RESET ","");
            } else if(drive.rightSlide.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) &&
                    drive.leftSlide.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER) &&
                    gamepad2.left_stick_y == 0) {
                drive.rightSlide.setPower(0);
                drive.leftSlide.setPower(0);
                yPressed = false;
//                telemetry.addData("ZERO POWER ","");
            }else{
//                telemetry.addData("NOTHING ","");
            }
//            telemetry.update();

            upPressed = gamepad2.dpad_right;
            downPressed = gamepad2.dpad_left;

            if (upPressed && !upLast1) {
                position = position + 100;
                position = Range.clip(position,0,850);
            }
            upLast1 = upPressed;

            if (downPressed && !downLast) {
                position = position - 100;
                position = Range.clip(position,0,850);
            }
            downLast = downPressed;

            Math.abs(drive.lifter.getCurrentPosition());
            if (gamepad2.dpad_up) {
                drive.planeLift.setPosition(.49);
            } else if (gamepad2.dpad_down || drive.lifter.getCurrentPosition() >= -2000) {
                drive.planeLift.setPosition(.25);
//            } else if (drive.lifter.getCurrentPosition() < -7000) {
            } else if (drive.lifter.getCurrentPosition() < -1888) {
                drive.planeLift.setPosition(.5);
            }

            if (gamepad2.right_bumper) {
                drive.pixelScore.setPosition(.559);
            } else {
                drive.pixelScore.setPosition(.46);
            }

//            if (gamepad2.right_bumper) {
//                drive.pixelScore.setPosition(Range.clip(drive.pixelScore.getPosition()+.01,0,.559));
//            } else if(gamepad2.left_bumper) {
//                drive.pixelScore.setPosition(Range.clip(drive.pixelScore.getPosition()-.01,0,.559));
//            }

            if (gamepad2.left_stick_y != 0) {
                drive.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.lifter.setPower(gamepad2.left_stick_y);
            } else if (gamepad2.x) {
//                drive.lifter.setTargetPosition(-14000);
                drive.lifter.setTargetPosition(-3777);
                drive.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.lifter.setPower(1);
            } else if (gamepad2.left_stick_y == 0 && drive.lifter.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                drive.lifter.setPower(0);
            }

            if (gamepad1.a && gamepad2.a) {
                drive.plane.setPosition(1);
            } else {
                drive.plane.setPosition(0);
            }

            if (gamepad1.right_trigger != 0) {
                drive.intake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger != 0) {
                drive.intake.setPower(-gamepad1.left_trigger);
            } else if (gamepad2.right_trigger != 0) {
                drive.intake.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger != 0) {
                drive.intake.setPower(-gamepad2.left_trigger);
            } else {
                drive.intake.setPower(0);
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("uplast: ", upLast);
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Target Pos: ", position);
            telemetry.addData("Drop Down: ", drive.dropdown.getPosition());
            telemetry.addData("pixel score: ", drive.pixelScore.getPosition());
            telemetry.addData("pixel: ", drive.pixel.getPosition());
            telemetry.addData("plane lift: ", drive.planeLift.getPosition());
            telemetry.addData("lifter Busy? ", drive.lifter.isBusy());
            telemetry.addData("Right Slide ", drive.rightSlide.getCurrentPosition());
            telemetry.addData("Left Slide ", drive.leftSlide.getCurrentPosition());
            telemetry.addData("y Pressed ", yPressed);
            telemetry.addData("pixels:",pixelCounter.pixels);
            telemetry.addData("isPixels:",pixelCounter.isPixel);
            telemetry.update();
        }
    }

    public void extendSlides(Drive drive, double speed, int position) {
        drive.rightSlide.setTargetPosition(position);
        drive.leftSlide.setTargetPosition(position);

        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.rightSlide.setPower(speed);
        drive.leftSlide.setPower(speed);
    }

    public void retractSlides(Drive drive, double speed) {
//        autoRetractSlides = true;
        drive.rightSlide.setTargetPosition(10);
        drive.leftSlide.setTargetPosition(10);

        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.rightSlide.setPower(speed);
        drive.leftSlide.setPower(speed);
    }

    public void areSlidesRetractedThenReset(Drive drive){
        if(drive.rightSlide.getCurrentPosition() < 10 && drive.leftSlide.getCurrentPosition() < 10)

//            && drive.rightSlide.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION) &&
//            drive.leftSlide.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION) &&
//            autoRetractSlides == true)
            {
            drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            autoRetractSlides = false;
            }
    }

    public void stopResetSlides(Drive drive){
        drive.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.rightSlide.setPower(0);
        drive.leftSlide.setPower(0);
    }
}

