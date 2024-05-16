package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.opencv.imgcodecs.Imgcodecs.imread;

import android.graphics.ColorSpace;

import org.opencv.core.Mat;

import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class OpenCvPipelineTest extends OpenCvPipeline
{
    double[] colorC = new double[2];
    Scalar color;
    Mat x = new Mat();

    double val[] = new double[4];

    @Override
    public Mat processFrame(Mat input)
    {
        x = input;

        int w = input.cols();
        int h = input.rows();

        Imgproc.cvtColor(x , x , Imgproc.COLOR_RGB2HSV);

        colorC = x.get(h/2 , w/2 ); /// ia culorile
        color = new Scalar(colorC); /// face culorile intr un scalar

        val[0] = color.val[0]; /// hue (culoarea in sine?)
        val[1] = color.val[1]; /// saturatie (cate de mult gri e in imagine)
        val[2] = color.val[2]; /// value (cat de luminoasa e culoarea)
        val[3] = color.val[3]; /// idk



        for(int i = 0 ; i<x.rows()-8; i+=5)
            for(int j = 0 ; j<x.cols() ; j+=5) {
                colorC = input.get(i , j );         /// ia culorile din frame
                color = new Scalar(colorC);
                if(color.val[0]>7 && color.val[0]<11) {
                    Imgproc.line(x, new Point(j, i), new Point(j, i), new Scalar(0, 0, 0), 15);
                }
            }


//        Mat c = x;
//        for(int i = 1 ; i<x.rows()-1 ; i++)
//            for(int j = 0 ; j<x.cols() ; j++)
//            {
//                left   = new double[1];
//                right  = new double[1];
//                center = new double[1];
//                inputCl = new double[1];
//
//                left = c.get(i-1 , j);
//                right = c.get(i+1 , j);
//                center = x.get( i , j);
//                inputCl = input.get( i , j);
//
//                colorLeft = new Scalar(left);
//                colorRight = new Scalar(right);
//                colorCenter = new Scalar(center);
//                inputColor = new Scalar(inputCl);
//
//                if(colorCenter.val[0] != colorLeft.val[0] || colorCenter.val[0] != colorRight.val[0] )
//                    Imgproc.line(x , new Point(j , i) , new Point(j , i), inputColor , 1);
//            }
//        Imgproc.erode( x , x , kernel);
//        Imgproc.erode( x , x , kernel);
//        Imgproc.rectangle(x , new Point(left , up) , new Point( right , down) , new Scalar( 100 , 25 , 25) , 2);
        Imgproc.cvtColor(x ,x ,Imgproc.COLOR_HSV2RGB);

        return x; /// da return la imaginea prelucrata
    }
    public double[] getValues()
    {
        return val; /// scoate valorile din pipeline
    }
    public double getSizeOFMat(Mat mat){
        return mat.rows()*10000+mat.cols();
    }
    public Mat getMat()
    {
        return x;
    }
}