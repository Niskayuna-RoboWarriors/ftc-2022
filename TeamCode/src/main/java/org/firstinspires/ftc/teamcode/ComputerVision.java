/* Author: Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import com.google.gson.JsonArray;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.*;

import androidx.annotation.Nullable;
import android.os.Environment;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import java.util.*;

import org.opencv.android.Utils;
import android.graphics.BitmapFactory;
import android.graphics.Bitmap;

import org.opencv.core.*;

import org.opencv.features2d.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.calib3d.Calib3d;

import org.openftc.easyopencv.*;


/** Managing class for opening cameras, attaching pipelines, and beginning streaming.
 */
public class ComputerVision {

    public OpenCvCamera camera;
    public OpenCvPipeline pipeline;

    public static String DataDir = Environment.getExternalStorageDirectory().getAbsolutePath() + "/FIRST/cvdata";


    ComputerVision(HardwareMap hardwareMap, OpenCvPipeline pipeline) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        this.pipeline = pipeline;
    }


    /** Begins continuous frame acquisition and image processing.
     */
    public void startStreaming() {
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
//            public void onOpened() {
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }


            // TODO: responsible error handling
            @Override
            public void onError(int errorCode) {}
        });
    }

    public void stopStreaming() {
        camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {}
        });
    }
}




/** Contains all image processing done for scanning the barcode and getting position.
 * This will
 */
class AutonPipeline extends OpenCvPipeline {
    final private Robot robot;
    final private Telemetry telemetry;
    final private Mat output;  // Frame to be displayed on the phone
    final private RobotManager.AllianceColor allianceColor;

    boolean first = true;


    AutonPipeline(Robot robot, Telemetry telemetry, RobotManager.AllianceColor allianceColor) {
        super();
        this.robot = robot;

        // TODO: there might be a cleaner way to default-initialize these
        output = new Mat();


        //Initialize signal data
        signalHsv = new Mat();

        signalRegionsRed = new Mat();
        signalRegionsGreen = new Mat();
        signalRegionsBlue = new Mat();

        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
    }


    /**
     * The main pipeline method, called whenever a frame is received. It will execute all necessary CV tasks, such as localization and barcode scanning
     *
     * @param input The current frame read from the attached camera.
     *              NOTE: the camera will be mounted in landscape, so make sure to flip x/y coords
     * @return An output frame to be displayed on the phone
     */
    @Override
    public Mat processFrame(Mat input) {
//        return input;
////        if (first) {
//            saveMatToDiskFullPath(input, ComputerVision.DataDir + "/firstimage.png");
//            first = false;
//        }


        // Check if a signal scan has been requested
        if (robot.signalScanState == Robot.SignalScanState.SCAN) {
            // Scan the signal
            input.copyTo(output);
            Robot.SignalScanResult result = processSignalFrame(input, output);

            if (robot.numSignalAttempts % 5 == 0) {
                saveMatToDiskFullPath(output, ComputerVision.DataDir + "/barcodeImage" + robot.numSignalAttempts + ".png");
            }

            // Increment the barcode result in the frequency counter and find the max value in that map
            int freq = robot.signalScanResultMap.get(result);
            robot.signalScanResultMap.put(result, freq + 1);

            Map.Entry<Robot.SignalScanResult, Integer> max = Collections.max(robot.signalScanResultMap.entrySet(), Comparator.comparingInt(Map.Entry::getValue));
            robot.numSignalAttempts++;

            if (robot.numSignalAttempts >= Robot.MAX_SIGNAL_ATTEMPTS || max.getValue() >= Robot.MIN_SIGNAL_REPEAT) {
                Map<Robot.SignalScanResult, Integer> fullResultMap = robot.signalScanResultMap;

                robot.signalScanResult = max.getKey();
                robot.signalScanState = Robot.SignalScanState.CHECK_SCAN;
                robot.signalScanResultMap = fullResultMap;
            } else {
                return input;  // We have more iterations of barcode scanning to do, so we needn't spend time on positioning
            }
        }

//        Position currentPosition = processPositioningFrame(input, output);
//        if (currentPosition != null) robot.positionManager.updateCvPosition(currentPosition);

        return input;
    }


    // BARCODE SCANNING
    // =================

    // Single-time allocated mats that will hold frame processing data
    final private Mat signalHsv;
    final private Mat signalRegionsRed, signalRegionsGreen, signalRegionsBlue;

    // The Region of Interest that contains all the barcode elements and the least non-floor background possible
    final static int SIGNAL_CROP_LEFT = 150;
    final static int SIGNAL_CROP_TOP = 180;
    final static int SIGNAL_CROP_RIGHT = 30;

    // Define HSV scalars that represent ranges of color to be selected from the barcode image
    //TODO: change these colors to match with actual signal colors!!!
    final static Scalar[] SIGNAL_RANGE_RED      = {new Scalar(23, 60, 50), new Scalar(78, 255, 255)};
    final static Scalar[] SIGNAL_RANGE_GREEN = {new Scalar(100, 100, 50), new Scalar(120, 255, 255)};
    final static Scalar[] SIGNAL_RANGE_BLUE = {new Scalar(170, 100, 50), new Scalar(180, 255, 255)};

    static final Size NOISE_SIZE = new Size(5, 5);

    /** Isolates the sections of an image in a given HSV range and removes noise, to find large solid-color areas
     * @param hsv The input image to be isolated, in HSV color format
     * @param out The image in which the detected areas will be stored
     * @param a HSV color in Scalar format that represents the lower bound of the area to be isolated
     * @param b HSV color in Scalar format that represents the upper bound of the area to be isolated
     * NOTE: OpenCV represents hue from 0-180
     */
    private static void isolateSignalRange(Mat hsv, Mat out, Scalar a, Scalar b) {
        Core.inRange(hsv, a, b, out);

        //Noise reduction
        Imgproc.morphologyEx(out, out, Imgproc.MORPH_CLOSE, Mat.ones(NOISE_SIZE, CvType.CV_32F));
        Imgproc.morphologyEx(out, out, Imgproc.MORPH_OPEN, Mat.ones(NOISE_SIZE, CvType.CV_32F));
    }


    private static class BarcodeCentroid implements Comparable<BarcodeCentroid> {
        BarcodeCentroid(int index, double centroidX) {
            this.index = index;
            this.centroidX = centroidX;
        }

        @Override
        public int compareTo(BarcodeCentroid other) {
            return Double.compare(this.centroidX, other.centroidX);
        }

        public int index;
        public double centroidX;
    };


    public static int[] BarcodeFlags = {235, 415, 570};


    /**
     * @param input The current frame containing the barcode to be scanned
     * @return an integer in the interval [-1, 2], where -1 denotes no result, and 0-2 represent positions (in screen space) of the object of interest
     */
    private Robot.SignalScanResult processSignalFrame(Mat input, Mat output) {
        Mat frame = input.submat(new Rect(0, SIGNAL_CROP_LEFT, SIGNAL_CROP_TOP, input.rows() - SIGNAL_CROP_LEFT));

        // Convert input image to HSV space and perform basic blur
        Imgproc.cvtColor(frame, signalHsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(signalHsv, signalHsv, new Size(7, 7), 5);

        // HSV thresholding for signal type classification
        isolateSignalRange(signalHsv, signalRegionsRed, SIGNAL_RANGE_RED[0], SIGNAL_RANGE_RED[1]);
        isolateSignalRange(signalHsv, signalRegionsGreen, SIGNAL_RANGE_GREEN[0], SIGNAL_RANGE_GREEN[1]);
        isolateSignalRange(signalHsv, signalRegionsBlue, SIGNAL_RANGE_BLUE[0], SIGNAL_RANGE_BLUE[1]);


//         Visualize the detected areas with appropriately colored outlines
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(signalRegionsRed, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Draw the detected areas to the output for visualization
        // TODO: Get rid of this for competition

        input.copyTo(output);

        //If this still draws stuff, it should be drawing only red regions
        for (int idx = 0; idx < contours.size(); idx++) {
            Imgproc.drawContours(output, contours, idx, new Scalar(255, 255, 0), 6);
        }

        contours.clear();
        Imgproc.findContours(signalRegionsRed, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        for (int idx = 0; idx < contours.size(); idx++) {
            Imgproc.drawContours(output, contours, idx, new Scalar(255, 0, 0), 6);
        }

        //count number of masked pixels in each signal isolation, see which one has the most
        int redCount = 0;
        int greenCount = 0;
        int blueCount = 0;
        //signal regions could go into array to make this more efficient
        for(int row = 0; row < signalRegionsRed.rows(); row ++) {
            for(int col = 0; col < signalRegionsRed.cols(); col ++) {
                int red = (int) signalRegionsRed.get(row, col)[0]; //just checking red for simplicity
                redCount += red == 255 ? 1 : 0;
            }
        }
        for(int row = 0; row < signalRegionsGreen.rows(); row ++) {
            for(int col = 0; col < signalRegionsGreen.cols(); col ++) {
                int red = (int) signalRegionsGreen.get(row, col)[0]; //just checking red for simplicity
                greenCount += red == 255 ? 1 : 0;
            }
        }
        for(int row = 0; row < signalRegionsBlue.rows(); row ++) {
            for(int col = 0; col < signalRegionsBlue.cols(); col ++) {
                int red = (int) signalRegionsBlue.get(row, col)[0]; //just checking red for simplicity
                blueCount += red == 255 ? 1 : 0;
            }
        }

        Robot.SignalScanResult result;
        int maxCount = Math.max(redCount, Math.max(greenCount, blueCount));
        if(maxCount == redCount) {
            result = Robot.SignalScanResult.SIGNAL3;
        } else if(maxCount == greenCount) {
            result = Robot.SignalScanResult.SIGNAL2;
        } else {
            result = Robot.SignalScanResult.SIGNAL1;
        }

        return result;
    }


    @Override
    public void onViewportTapped() {
//        camera.pauseViewport();
    }
}