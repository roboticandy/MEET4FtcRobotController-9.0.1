/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Actual;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;

/*
 * This OpMode shows how to use the new universal IMU interface. This
 * interface may be used with the BNO055 IMU or the BHI260 IMU. It assumes that an IMU is configured
 * on the robot with the name "imu".
 *
 * The sample will display the current Yaw, Pitch and Roll of the robot.<br>
 * With the correct orientation parameters selected, pitch/roll/yaw should act as follows:
 *   Pitch value should INCREASE as the robot is tipped UP at the front. (Rotation about X) <br>
 *   Roll value should INCREASE as the robot is tipped UP at the left side. (Rotation about Y) <br>
 *   Yaw value should INCREASE as the robot is rotated Counter Clockwise. (Rotation about Z) <br>
 *
 * The yaw can be reset (to zero) by pressing the Y button on the gamepad (Triangle on a PS4 controller)
 *
 * This specific sample assumes that the Hub is mounted on one of the three orthogonal planes
 * (X/Y, X/Z or Y/Z) and that the Hub has only been rotated in a range of 90 degree increments.
 *
 * Note: if your Hub is mounted on a surface angled at some non-90 Degree multiple (like 30) look at
 *       the alternative SensorImuNonOrthogonal sample in this folder.
 *
 * This "Orthogonal" requirement means that:
 *
 * 1) The Logo printed on the top of the Hub can ONLY be pointing in one of six directions:
 *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
 *
 * 2) The USB ports can only be pointing in one of the same six directions:<br>
 *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
 *
 * So, To fully define how your Hub is mounted to the robot, you must simply specify:<br>
 *    logoFacingDirection<br>
 *    usbFacingDirection
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * Finally, choose the two correct parameters to define how your Hub is mounted and edit this OpMode
 * to use those parameters.
 */
@TeleOp(name = "Slides", group = "Slids")
@Disabled
public class Slides extends LinearOpMode
{
    // The IMU sensor object
    IMU imu;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap);
        int position = 250;
        boolean upPressed = false;
        boolean downPressed = false;
        boolean upLast = false;
        boolean downLast = false;

        // Loop and update the dashboard
        while (!isStopRequested()) {
            if (gamepad1.a) {
                extendSlidesNew(drive, .8, position);
                drive.pixelScore.setPosition(1);
                while(drive.rightSlide.isBusy()){
                    telemetry.addData("L/R", drive.rightSlide.getCurrentPosition() + "/" + drive.leftSlide.getCurrentPosition());
                    telemetry.update();
                }
            }
            if (gamepad1.right_stick_y != 0) {
                drive.rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.rightSlide.setPower(-gamepad1.right_stick_y);
                drive.leftSlide.setPower(-gamepad1.right_stick_y);
            } else {

//                drive.rightSlide.setPower(0);
//                drive.leftSlide.setPower(0);
                drive.pixelScore.setPosition(0);
            }

            upPressed = gamepad1.dpad_up;
            downPressed = gamepad1.dpad_down;

            if(upPressed && !upLast){
                position = position + 25;
            }
            upLast = upPressed;

            if(downPressed && !downLast){
                position = position - 25;
            }

            downLast = downPressed;

            telemetry.addData("set point:", position);
            telemetry.addData("L/R", drive.rightSlide.getCurrentPosition() + "/" + drive.leftSlide.getCurrentPosition());
            telemetry.update();
        }
    }
    public void extendSlidesNew(Drive drive, double speed, int position){
        drive.pixelScore.setPosition(0);

        drive.rightSlide.setTargetPosition(position);
        drive.leftSlide.setTargetPosition(position);

        drive.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive.rightSlide.setPower(speed);
        drive.leftSlide.setPower(speed);
    }
}
