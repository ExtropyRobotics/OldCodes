

package org.firstinspires.ftc.teamcode.AutoSomes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import com.qualcomm.robotcore.util.BatteryChecker;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name = "Concept_TensorFlow_Daniel_inainte_de_OPMode", group = "Concept")


public class Concept_TensorFlow_Daniel_inainte_de_OPMode extends LinearOpMode {
  
  	DcMotor motorStangaFata = null;
		DcMotor motorStangaSpate = null;
		DcMotor motorDreaptaFata = null;
		DcMotor motorDreaptaSpate = null;
		DcMotor motorManuta = null;
		Servo servo1 = null;
		Servo servo2 = null;
		Rev2mDistanceSensor CS1 = null , CS2 = null;
		DcMotor planetara1 = null;
		DcMotor planetara2 = null;
		
		

	private final String TFOD_MODEL_ASSET = "model_20220609_121651.tflite";
    private final String[] LABELS = {
	 "CON"
    };

   
    public final String VUFORIA_KEY ="AcVF5pL/////AAABmZd7iOoOwUXzsbtrqWxbvY1n0R/mId58iOk4+lTHeExyLnua0bSzHpnQyZkm0Oal8xU8TU6IVWctYdetSpKYLXCEtaXOqDU+t6MtdIMqeiJ+cqaneXEmO74im1ghXc5y/J7VZ+9ocgtNEe3Fvxy1izAhsYjQemLWAXA7etsQaEcuRaFU6K3OtaMTPvsMz+armuUfuc6dbjZTUWkwooR5Grj7WDBb+4s6KK2t3WSqQ1AYLwIx/AbT+3D75HombGPeoIvwnCy7rE1RlmDocIhx+TG1pKhnVaYXVdNgtYrbglRXBy5TV6PgmsvTx4h2gHVKnp/Z/omtjvzYz0bqgq0fnaRffvRNZjP3+9voN9EzqIT2";


   
    public VuforiaLocalizer vuforia ;

  
    public TFObjectDetector tfod ;
    
    



	
	
	   



		
		
		
		
		
		
		 
		
		
		
		
public void RunEncoders()
{
		motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorManuta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
}
public void ResetEncoders()
{
		motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
}
public void SetPosition(int target)
{
		motorStangaFata.setTargetPosition(target);
		motorStangaSpate.setTargetPosition(target);
		motorDreaptaFata.setTargetPosition(target);
		motorDreaptaSpate.setTargetPosition(target);
}
public void SetPower(double power)
{
	  
		motorStangaFata.setPower(power);
		motorDreaptaFata.setPower(power);
		motorStangaSpate.setPower(power);
		motorDreaptaSpate.setPower(power);
}
public void RunPosition()
{
		motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		
}
	public void Forward(int target , float power , int rotation)
	{
		RunEncoders();
		ResetEncoders();
		SetPosition(target*rotation);
		SetPower(power);
		RunPosition();
		sleep(1200);
		SetPower(0);
	}
	public void Back(int target , float power , int rotation)
	{
		RunEncoders();
		ResetEncoders();
		SetPosition(-target*rotation);
		SetPower(power);
		RunPosition();
		sleep(1000);
		SetPower(0);
	}
	public void Rotate(int target , double power , int sleep_time)
	{
		RunEncoders();
		ResetEncoders();
		motorDreaptaFata.setTargetPosition(target*105);
		motorStangaFata.setTargetPosition(-target*105);
		motorDreaptaSpate.setTargetPosition(target*105);
		motorStangaSpate.setTargetPosition(-target*105);
		SetPower(power);
		RunPosition();
		sleep(sleep_time);
		SetPower(0);
		
	}
	public void Planetare(float power , int time)
	{
		planetara1.setPower(power);
		planetara2.setPower(power);
		sleep(time);
		
		planetara1.setPower(0);
		planetara2.setPower(0);
		
	}
	public void Strafe(int target , float power , int sleep_time)
	{
		RunEncoders();
		ResetEncoders();
		motorDreaptaFata.setTargetPosition(-target*250);
		motorStangaSpate.setTargetPosition(-target*250);
		motorDreaptaSpate.setTargetPosition(target*250);
		motorStangaFata.setTargetPosition(target*250);
		SetPower(power);
		RunPosition();
		sleep(sleep_time);
		SetPower(0);
	}
	public void Manuta(int target)
	{
		motorManuta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorManuta.setTargetPosition(target*50);
		motorManuta.setPower(0.5f);
		motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		sleep(2500);
		motorManuta.setPower(0);
	}
	public void Servo(double position)
	{
		if(position == 1)
		{
		servo1.setPosition(0.5);
		servo2.setPosition(0);
		}
		else
		{
		servo1.setPosition(0.3);
		servo2.setPosition(0.3);
		}
		
	}
	public void Caz_11(double left ,double right)
	{
		ResetEncoders();
		
		telemetry.addData("Caz 1 left" , left);
		telemetry.addData("right " , right);
		telemetry.update();
		Forward(2 , 0.7f , 350);
		sleep(100);
		Manuta(23);
		Servo(1);
		sleep(100);
		Rotate(1 , 0.5f , 300);
		Strafe(9 , 0.6f , 1200);
		sleep(100);
		Servo(0);
		sleep(100);
		Rotate(2 , 0.3f ,300);
		Manuta(-25);
		sleep(100);
		Strafe(-23 , 0.6f , 2300);
		sleep(100);
		Rotate(7 , 0.5f , 1000);
		Strafe(-2 , 0.7f , 800);
		sleep(100);
		Planetare(-0.7f , 3000);
		sleep(100);
		Strafe(9 , 0.5f , 1000);
		Forward(3, 0.5f , 250);
		Servo(1);
		
	}
	public void Caz_22(double left , double right)
	{
		ResetEncoders();
		
		telemetry.addData("Caz 2 left" , left);
		telemetry.addData("right " , right);
		telemetry.update();
		sleep(500);
		Forward(2 , 0.7f , 290);
		Manuta(27);
		sleep(100);
		Rotate(1 , 0.5f , 300);
		Strafe(7 , 0.7f , 1100);
		sleep(100);
		Servo(0);
		Rotate(1 , 0.35f ,350);
		Forward(-1 , 0.5f , 200);
		Manuta(-30);
		
		Strafe(-21 , 0.6f , 2200);
		
		Rotate(7 , 0.55f , 1000);
		sleep(100);
		Strafe(-2 , 0.5f , 750);
		sleep(100);
		Planetare(-0.7f , 3000);
		sleep(100);
		Strafe(9 , 0.5f , 1000);
		Forward(3, 0.5f , 260);
		Servo(1);
		
	}
	
	public void Caz_33(double left , double right)
	{
		ResetEncoders();
		
		telemetry.addData("Caz 3 left" , left);
		telemetry.addData("right " , right);
		telemetry.update();
		sleep(500);
		Forward(2 , 0.6f , 300);
		Manuta(31);
		sleep(100);
		Rotate(1 , 0.6f , 200);
		Strafe(10 , 0.6f , 1100);
		sleep(100);
		Servo(0);
		Rotate(2 , 0.35f ,350);
		Manuta(-33);
		sleep(100);
		Strafe(-17 , 0.75f , 2000);
		sleep(100);
		Rotate(7 , 0.5f , 1000);
		sleep(100);
		Strafe(-2 , 0.5f , 300);
		Planetare(-0.7f , 5000);
		sleep(100);
		Strafe(9 , 0.5f , 900);
		Forward(3, 0.5f , 270);
		Servo(1);
		
	}

private void initVuforia() {
	   
	   VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

	   parameters.vuforiaLicenseKey = VUFORIA_KEY;
	   parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

	   //  Instantiate the Vuforia engine
	   vuforia = ClassFactory.getInstance().createVuforia(parameters);

	   // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
	* Initialize the TensorFlow Object Detection engine.
	*/
    private void initTfod() {
	   int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
		  "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
	   TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
	  tfodParameters.minResultConfidence = 0.8f;
	  tfodParameters.isModelTensorFlow2 = true;
	  tfodParameters.inputSize = 320;
	  tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
	  tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
		
	


   class Camera implements Runnable{
    	public List<Recognition> finalus;
    	public List<Recognition> us;
    	Camera()
    	{
    		initVuforia();
    		initTfod();
    		if (tfod != null) {
		  tfod.activate();
		  tfod.setZoom(1, 16.0/9.0);
	   }
    		
    	}
    	public void run()
    	{
    		boolean f = true;
    		while (f) {
			 if (tfod != null) {
				// getUpdatedRecognitions() will return null if no new information is available since
				// the last time that call was made.
				us = tfod.getUpdatedRecognitions();
				if (us != null) {
				  telemetry.addData("# Object Detected", us.size());
				  // step through the list of recognitions and display boundary info.
				  int i = 0;
				  for (Recognition recognition : us) {
				    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
				    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
						  recognition.getLeft(), recognition.getTop());
				    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
						  recognition.getRight(), recognition.getBottom());
				    i++;
				    if(opModeIsActive())
				    {
				    	f = false;
				    }
				  }
				  telemetry.update();
				}
			 }
		  }
		  finalus = us;
    	}
    	
    	
    }


    @Override
    public void runOpMode() throws InterruptedException {
	   
	Camera cam = new Camera();
	Thread thread = new Thread(cam);
	   
	   
		motorStangaFata = hardwareMap.get(DcMotor.class , "motorStangaFata");
		motorStangaSpate = hardwareMap.get(DcMotor.class , "motorStangaSpate");
		motorDreaptaFata = hardwareMap.get(DcMotor.class , "motorDreaptaFata");;
		motorDreaptaSpate = hardwareMap.get(DcMotor.class , "motorDreaptaSpate");
		motorManuta = hardwareMap.get(DcMotor.class , "motorManuta");
		servo1 = hardwareMap.get(Servo.class , "servo1");
		servo2 = hardwareMap.get(Servo.class , "servo2");
		planetara1 = hardwareMap.get(DcMotor.class , "planetara1");
		planetara2 = hardwareMap.get(DcMotor.class , "planetara2");
		
		motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
		motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
		
		motorDreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorStangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorDreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorStangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motorManuta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
	  
	   
	   
	   
	   
	   double leftCorner  = 0;
	   double rightCorner = 0;
	   
	   telemetry.addData(">", "Camera Active press play to start OpMode");
	   telemetry.update();
	   
	   thread.start();
	   
	   waitForStart();
	
	   
	    sleep(3000);
	    
	   
	   
	   
	   telemetry.addLine("Started");
	   telemetry.update();
	   sleep(1000);
	   
	   
	  
RunEncoders();
ResetEncoders();
Servo(1);






if(opModeIsActive()){
	if(cam.finalus.size()>=2)
	{
		leftCorner =cam.finalus.get(1).getLeft();
		
	}
	else if(cam.us.size()>=1){
		leftCorner =cam.finalus.get(0).getLeft();
		
	}
	else
	{
		leftCorner = 300;
	}
	if(isStopRequested()!=true){
	
if(cam.finalus.size()>=1){
	
	if(leftCorner<300)
	{
		
		Caz_33(leftCorner , rightCorner);
		
	}
	else
	if(leftCorner>300)
	{
		
		Caz_22(leftCorner , rightCorner);
		
	}
	
}
else
{
		
		Caz_11(leftCorner , rightCorner);
		
}
}}
    	
    }
	
	

}
	   
	   
			 

    /**
	* Initialize the Vuforia localization engine.
	*/
    

