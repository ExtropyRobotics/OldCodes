package org.firstinspires.ftc.teamcode.camera;

import static org.opencv.core.CvType.CV_8UC1;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OrangePipeline extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input)
    {
        Mat oneColor = new Mat(); // oneColor from image but white
        Mat colored = new Mat(); // oneColor but recolored
        Mat avrColored = new Mat(); // used for removing imperfections when u average the saturation
        Mat finMask = new Mat();
        Mat finImage = new Mat();
        Mat eroded = new Mat();
        Mat edges = new Mat();

        Scalar low = new Scalar(6 , 100 , 40); ///low portocaliu
        Scalar high = new Scalar(12, 255, 255); ///high portocaliu

        Imgproc.cvtColor(input , input , Imgproc.COLOR_RGB2HSV);
        Core.inRange(input , low , high , oneColor); /// select only orange but white now
        Core.bitwise_and(input , input , colored , oneColor); /// recolor the white stuff

        Scalar average = Core.mean(colored , oneColor); /// average the saturation on the colored image
        colored.convertTo(avrColored , -1 , 120/average.val[1] , 0);

        Scalar strictLow = new Scalar(0 , 120 , 50);
        Scalar strictHigh = new Scalar(255 , 255 , 255);

        Core.inRange(colored , strictLow , strictHigh , finMask); /// apply the stricter saturation range
        Core.bitwise_and(input , input , finImage , finMask); /// recolor the new finMask in finImage

        finImage.copyTo(input);

        Imgproc.cvtColor(input , input , Imgproc.COLOR_HSV2RGB); // turn back to rgb , dont with edges
        Imgproc.cvtColor(input , input , Imgproc.COLOR_RGB2GRAY);

        Mat kernelOpen = Mat.ones( 3 , 3 , CV_8UC1); /// apply a blur of 3x3
        Mat kernelClose = Mat.ones( 9 , 9 , CV_8UC1); /// apply a blur of 5x5

        /// get rid of noise in general

        Imgproc.morphologyEx(input , eroded , Imgproc.MORPH_OPEN , kernelOpen);
        Imgproc.morphologyEx(input , eroded , Imgproc.MORPH_OPEN , kernelOpen);

        Imgproc.morphologyEx(eroded , eroded , Imgproc.MORPH_CLOSE , kernelClose);
        Imgproc.morphologyEx(eroded , eroded , Imgproc.MORPH_CLOSE , kernelClose);

        eroded.copyTo(input);

        Imgproc.Canny(input , edges , 100 , 200); /// take the edges
        edges.copyTo(input);

        /// release mats to not cause memory leaks

        oneColor.release();
        colored.release();
        finImage.release();
        finMask.release();
        edges.release();
        eroded.release();
        kernelClose.release();
        kernelOpen.release();

        return input;
    }

}
