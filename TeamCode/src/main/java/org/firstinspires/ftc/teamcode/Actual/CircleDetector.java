package org.firstinspires.ftc.teamcode.Actual;

import android.provider.ContactsContract;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Vector;

public class CircleDetector extends OpenCvPipeline {


    boolean viewportPaused;
    Mat gray = new Mat();
    Mat circles = new Mat();
    Vector<Mat> circlesList = new Vector<Mat>();

    Point centerOfProp = new Point(-1, -1);
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.medianBlur(gray, gray, 5);

        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1, gray.rows() / 16, 100, 30, 15, 100);

        double x = 0.0;
        double y = 0.0;
        int r = 0;
        for (int i = 0; i < circles.cols(); i++) {
            double[] data = circles.get(0, i);
            for (int j = 0; j < data.length; j++) {
                x = data[0];
                y = data[1];
                r = (int) data[2];
            }
            centerOfProp = new Point(x, y);
            // circle center
            Imgproc.circle(input, centerOfProp, 3, new Scalar(0, 255, 0), -1);
            // circle outlin
            Imgproc.circle(input, centerOfProp, r, new Scalar(0, 0, 255), 1);
            Imgproc.putText(input, String.valueOf(r), centerOfProp, 1, 1, new Scalar(255, 255, 255));
            Imgproc.putText(input, String.valueOf(x), new Point(x + 25, y), 1, 1, new Scalar(255, 255, 255));
        }

        Imgproc.putText(input, String.valueOf(circles.cols()), new Point(240, 160), 1, 1, new Scalar(255, 255, 255));
        if (circles.cols() == 0) {
            centerOfProp = new Point(0, 0);
        }

        circles.release();

        return gray;
    }
}
