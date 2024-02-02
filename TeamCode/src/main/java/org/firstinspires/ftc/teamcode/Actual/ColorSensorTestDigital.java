package org.firstinspires.ftc.teamcode.Actual;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous
public class ColorSensorTestDigital extends LinearOpMode {
    public static int propPos = 1;

    @Override
    public void runOpMode() {
        Rev2mDistanceSensor sensor;
//        AnalogInput sensor;
        sensor = hardwareMap.get(Rev2mDistanceSensor.class, "color");
//        sensor = hardwareMap.get(AnalogInput.class, "DistanceSensor");
        int pixels = 0;
        boolean isPixel = true;
        while (!isStarted()) {
//            telemetry.addLine(String.valueOf(sensor.getVoltage()));
//            System.out.println("Voltage" + sensor.getVoltage());
            System.out.println("Distance: " + sensor.getDistance(DistanceUnit.CM));
//            telemetry.addLine(String.valueOf(sensor.getRawLightDetected()));
//            if (sensor.getVoltage() > .04 && !isPixel) {
            pixels = pixels + 1;
            isPixel = true;
//            } else if (sensor.getVoltage() < .04 && isPixel) {
            isPixel = false;

//            telemetry.addLine(String.valueOf(pixels));
            telemetry.update();
        }
    }
}


