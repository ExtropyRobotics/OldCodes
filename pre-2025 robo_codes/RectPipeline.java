package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.opencv.core.Core.FILLED;
import static org.opencv.core.Core.minMaxLoc;
import static org.opencv.core.CvType.CV_8UC4;
import static org.opencv.imgcodecs.Imgcodecs.IMREAD_COLOR;
import static org.opencv.imgcodecs.Imgcodecs.IMREAD_GRAYSCALE;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.List;

public class RectPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat partOfObj;

    public RectPipeline(Telemetry tele)
    {
        telemetry = tele;

        telemetry.addLine("loading...");
        telemetry.update();

        partOfObj = Imgcodecs.imread("/sdcard/partOfObj.jpg", IMREAD_GRAYSCALE);

        telemetry.addLine("loaded !");
        telemetry.update();

    }
    @Override
    public void init(Mat firstFrame)
    {
        while(firstFrame == null)sleep(100);
    }
    @Override
    public Mat processFrame(Mat input)
    {
        Mat map = new Mat();

        Imgproc.resize(partOfObj , partOfObj , new Size(input.cols()/8 , input.rows()/8));
        Imgproc.cvtColor(input , input , Imgproc.COLOR_RGB2GRAY);

        Imgproc.matchTemplate(input , partOfObj , map , Imgproc.TM_SQDIFF);
        Core.normalize(map , map , 0 , 255);

        map.convertTo(map , 0);
        Imgproc.resize(map , map ,  input.size());

        telemetry.addLine("after convert " + map.size());
        telemetry.update();

        input.release();
        return map;
    }
}
