package org.firstinspires.ftc.teamcode.camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class TestPipeline extends OpenCvPipeline
{
    @Override
    public Mat processFrame(Mat input){
        Mat img = input;
        Mat mask = new Mat();
        Mat output = new Mat();
        Scalar lower = new Scalar(115 , 100 , 40);
        Scalar higher = new Scalar(125 , 255 , 255);

        Imgproc.cvtColor(img , img , Imgproc.COLOR_BGR2HSV);
        Core.inRange(img , lower , higher , mask);
        Core.bitwise_and(img , img , output , mask);
        return output;
    }
}