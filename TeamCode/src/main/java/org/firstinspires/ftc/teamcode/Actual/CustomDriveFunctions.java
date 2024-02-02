package org.firstinspires.ftc.teamcode.Actual;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.teamcode.RoadRunnerInternal.drive.Drive;

public class CustomDriveFunctions{
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.945; // in
    public static double GEAR_RATIO = 1;
    public void pickupPixels(Drive drive, int cycle) throws InterruptedException{
        double cyclePos = .7;
        if(cycle == 1){
            cyclePos = .399;
        } else if(cycle == 2){
            cyclePos = .36;
        } else if(cycle == 3){
            cyclePos = .34;
        }
        drive.dropdown.setPosition(cyclePos);
        forward(drive, 3,.25);
        drive.intake.setPower(.25);
//        sleep(250);
        forward(drive, 3,-.25);
        drive.dropdown.setPosition(.7);
        drive.intake.setPower(1);
        forward(drive, 4,.25);
    }

    public void forward(Drive drive, double desiredDistance, double speed){
        double startDistance = encoderTicksToInches(Math.abs(drive.rightRear.getCurrentPosition()));
        double measuredDistance = encoderTicksToInches(Math.abs(drive.rightRear.getCurrentPosition() - startDistance));
        while(desiredDistance > measuredDistance) {
            drive.setMotorPowers(speed,speed,speed,speed);
            measuredDistance = encoderTicksToInches(Math.abs(drive.rightRear.getCurrentPosition() - startDistance));
        }
        drive.setMotorPowers(0,0,0,0);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

}
