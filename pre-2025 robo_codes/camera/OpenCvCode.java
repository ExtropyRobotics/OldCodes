package org.firstinspires.ftc.teamcode.camera;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "!!!!!!!!!OpenCvAndroidStudio")

public class OpenCvCode extends LinearOpMode {

    @Override
    public void runOpMode(){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        OrangePipeline pipelineOrange = new OrangePipeline();
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        RectPipeline pipelineRect = new RectPipeline(telemetry);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {                       /// 800 , 448 initial
                camera.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(pipelineRect);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("error");
                telemetry.update();
            }
        });

        waitForStart();

    }

}