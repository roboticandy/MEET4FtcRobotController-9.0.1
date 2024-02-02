package org.firstinspires.ftc.teamcode.Actual;

import static com.qualcomm.robotcore.hardware.EmbeddedControlHubModule.get;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;
import org.firstinspires.ftc.teamcode.RoadRunnerInternal.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class ColorSensorTest extends LinearOpMode {
    public static int propPos = 1;

    @Override
    public void runOpMode() {
//        Rev2mDistanceSensor sensor;
        AnalogInput sensor;
//        sensor = hardwareMap.get(Rev2mDistanceSensor.class, "color");
        sensor = hardwareMap.get(AnalogInput.class, "pixelSensor");
        int pixels = 0;
        boolean isPixel = true;
        while (!isStarted()) {
            telemetry.addLine(String.valueOf(sensor.getVoltage()));
            System.out.println("Voltage: " + sensor.getVoltage());
//            telemetry.addLine(String.valueOf(sensor.));
//            telemetry.addLine(String.valueOf(sensor.getRawLightDetected()));
            if (sensor.getVoltage() > .006 && !isPixel) {
                pixels = pixels + 1;
                isPixel = true;
            } else if (sensor.getVoltage() < .003 && isPixel) {
                isPixel = false;
            }
            telemetry.addLine(String.valueOf(pixels));
            telemetry.update();

        }
    }
}
