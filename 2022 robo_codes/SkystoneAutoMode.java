// package OpenCv;

// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// import org.openftc.easyopencv.OpenCvCamera;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvCameraRotation;
// import org.openftc.easyopencv.OpenCvInternalCamera;

// @Autonomous(name="Skystone Detecotor", group="Auto")
// public class SkystoneAutoMode extends LinearOpMode {
//	OpenCvCamera webcam;
//	@Override
//	public void runOpMode() throws InterruptedException {
//		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//	    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "vod"), cameraMonitorViewId);
	   
	   
//		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//	    {
//		   @Override
//		   public void onOpened()
//		   {
//			  webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
//		   }

//		   @Override
//		   public void onError(int errorCode) {}
//	    });
//	    SkystoneDetector detector = new SkystoneDetector(telemetry);
//	    webcam.setPipeline(detector);
//	    waitForStart();
//	    switch (detector.getLocation()) {
//		   case LEFT:
//			  // ...
//			  break;
//		   case RIGHT:
//			  // ...
//			  break;
//		   case NOT_FOUND:
//			  // ...
//	    }
//	    webcam.stopStreaming();
//	}
// }
