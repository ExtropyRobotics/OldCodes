package org.firstinspires.ftc.teamcode.AutoSomes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous (name = "TeoTest", group = "Autonomous")

public class TeoTest extends LinearOpMode{
    DcMotor motorDreaptaFata = null;
    DcMotor motorStangaFata = null;
    DcMotor motorDreaptaSpate = null;
    DcMotor motorStangaSpate = null;
    DcMotor motorManuta = null;
    Servo servo1 = null;
    Servo servo2 = null;
    DcMotor planetara1 = null;
    DcMotor planetara2 = null;
	
	public void useEncoders()
    {
	   motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void ForwardMult(int target){
	   useEncoders();
	   resetEncoders();
	   setTargetPosition(target);
	   setPowerForMotors(1);
	   runToPosition();
	   sleep(2000);
	   setPowerForMotors(0);
	   resetEncoders();

    }
    public void ForwardPutin(int target){
	   useEncoders();
	   resetEncoders();
	   setTargetPosition(target);
	   setPowerForMotors(0.6);
	   runToPosition();
	   sleep(1100);
	   setPowerForMotors(0);
	   resetEncoders();

    }
    public void resetEncoders()
    {
	   motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void Back(int distanta){
	   useEncoders();
	   resetEncoders();
	   setTargetPosition(-distanta);
	   setPowerForMotors(0.6);
	   runToPosition();
	   sleep(1000);
	   setPowerForMotors(0);
	   resetEncoders();

    }
    public void StrafeRight(int diagonalaSus, int diagonalaJos){
	   useEncoders();
	   resetEncoders();
	   setTargetPositionForStrafe(-diagonalaSus , diagonalaJos);
	   setPowerForMotors(0.7);
	   runToPosition();
	   sleep(2000);
	   setPowerForMotors(0);
	   resetEncoders();

    }
    public void StrafeLeft(int diagonalaSus, int diagonalaJos){
	   useEncoders();
	   resetEncoders();
	   setTargetPositionForStrafe(diagonalaSus , -diagonalaJos);
	   setPowerForMotors(0.6);
	   runToPosition();
	   sleep(2000);
	   setPowerForMotors(0);
	   resetEncoders();
	   
    }
    public void StrafeDiagonal(int diagonalaSus, int diagonalaJos){
	   useEncoders();
	   resetEncoders();
	   setTargetPositionForStrafe(diagonalaSus, diagonalaJos);
	   setPowerForMotors(0.6);
	   runToPosition();
	   sleep(2000);
	   setPowerForMotors(0);
	   resetEncoders();

    }
    public void Rotate(int dreapta, int stanga){
	   useEncoders();
	   resetEncoders();
	   motorDreaptaFata.setTargetPosition(dreapta);
	   motorDreaptaSpate.setTargetPosition(dreapta);
	   motorStangaFata.setTargetPosition(stanga);
	   motorStangaSpate.setTargetPosition(stanga);
	   setPowerForMotors(0.8);
	   runToPosition();
	   sleep(1300);
	   setPowerForMotors(0);
	   resetEncoders();

    }
    
    public void MotorManuta(int target, int numar){
    	motorManuta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorManuta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motorManuta.setTargetPosition(target*numar);
		motorManuta.setPower(0.7);
		motorManuta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motorManuta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		sleep(1100);
		motorManuta.setPower(0);
    }
    public void setTargetPosition(int target){
	   motorDreaptaFata.setTargetPosition(target);
	   motorDreaptaSpate.setTargetPosition(target);
	   motorStangaFata.setTargetPosition(target);
	   motorStangaSpate.setTargetPosition(target);
    }
    public void setTargetPositionForStrafe(int diagonalaSus, int diagonalaJos){
	   motorDreaptaFata.setTargetPosition(diagonalaSus);
	   motorDreaptaSpate.setTargetPosition(diagonalaJos);
	   motorStangaFata.setTargetPosition(diagonalaJos);
	   motorStangaSpate.setTargetPosition(diagonalaSus);
    }

    public void setPowerForMotors(double power){
	   motorDreaptaFata.setPower(power);
	   motorDreaptaSpate.setPower(power);
	   motorStangaFata.setPower(power);
	   motorStangaSpate.setPower(power);
    }
    public void runToPosition(){
	   motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	   motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	   motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	   motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
	  public void servo(double afine){
	    servo1.setPosition(-afine+0.5);
	    servo2.setPosition(afine);
	    sleep(1000);
	  }
	  
	  public void Marcel(int cartof){
	  	planetara1.setPower(cartof);
	  	planetara2.setPower(cartof);
	  	sleep(2500);
	  	planetara1.setPower(0);
	  	planetara2.setPower(0);
	  	
	  	
	  }
	  
	  private static final String TFOD_MODEL_ASSET = "model_20220609_121651.tflite";
    private static final String[] LABELS = {
	 "CON"
    };

   
    public static final String VUFORIA_KEY ="AcVF5pL/////AAABmZd7iOoOwUXzsbtrqWxbvY1n0R/mId58iOk4+lTHeExyLnua0bSzHpnQyZkm0Oal8xU8TU6IVWctYdetSpKYLXCEtaXOqDU+t6MtdIMqeiJ+cqaneXEmO74im1ghXc5y/J7VZ+9ocgtNEe3Fvxy1izAhsYjQemLWAXA7etsQaEcuRaFU6K3OtaMTPvsMz+armuUfuc6dbjZTUWkwooR5Grj7WDBb+4s6KK2t3WSqQ1AYLwIx/AbT+3D75HombGPeoIvwnCy7rE1RlmDocIhx+TG1pKhnVaYXVdNgtYrbglRXBy5TV6PgmsvTx4h2gHVKnp/Z/omtjvzYz0bqgq0fnaRffvRNZjP3+9voN9EzqIT2";


   
    public VuforiaLocalizer vuforia ;

  
    public TFObjectDetector tfod ;

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
	  tfodParameters.minResultConfidence = 0.7f;
	  tfodParameters.isModelTensorFlow2 = true;
	  tfodParameters.inputSize = 320;
	  tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
	  tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
    
    public void CazSus() {//
    	telemetry.addLine("CazSus");
    	telemetry.update();
    		servo(0);
    		ForwardPutin(210);
 		MotorManuta(598, 2);   		
    		Rotate(300,-300);
    		ForwardPutin(700);
    		servo(0.23);
    		Back(200);
    		Rotate(-1920,1920);
    		MotorManuta(-780,2);
    		ForwardPutin(200);
    		StrafeLeft(2000,2000);
    		ForwardPutin(123);
    		Marcel(500);
    		Back(760);
    		
    }
    public void CazMijloc(){//
    	telemetry.addLine("CazMijloc");
    	telemetry.update();
    		servo(0);
    		ForwardPutin(210);
	  	MotorManuta(657,2);
	  	sleep(150);
		Rotate(290,-290);
    		ForwardPutin(694);
    		servo(0.23);
    		Back(200);
    		MotorManuta(-15,2);
    		Rotate(-1890,1890);
    		MotorManuta(-750,2);
    		ForwardPutin(330);
    		StrafeLeft(2000,2000);
    		ForwardPutin(187);
    		Marcel(500);
    		Back(773);
    		StrafeLeft(370 , 370);
    }
    public void CazJos(){//
    	telemetry.addLine("CazJos");
    	telemetry.update();
    		servo(0);
    		ForwardPutin(210);
	  	MotorManuta(791,2);
	  	sleep(300);
		Rotate(320,-320);
    		ForwardPutin(650);
    		servo(0.23);
    		Back(200);
    		Rotate(-1958,1958);
    		MotorManuta(-770,2);
    		ForwardPutin(200);
    		StrafeLeft(1900,1900);
    		ForwardPutin(227);
    		Marcel(500);
    		Back(700);
    }
	
    @Override
    public void runOpMode()throws InterruptedException{
    
    
    
    motorDreaptaSpate = hardwareMap.get(DcMotor.class,"motorDreaptaSpate");
	motorDreaptaFata = hardwareMap.get(DcMotor.class,"motorDreaptaFata");
	motorStangaSpate = hardwareMap.get(DcMotor.class,"motorStangaSpate");
	motorStangaFata = hardwareMap.get(DcMotor.class,"motorStangaFata");
	motorManuta = hardwareMap.get(DcMotor.class,"motorManuta");
	servo1 = hardwareMap.get(Servo.class, "servo1");
	servo2 = hardwareMap.get(Servo.class, "servo2");
	planetara1 = hardwareMap.get(DcMotor.class, "planetara1");
	planetara2 = hardwareMap.get(DcMotor.class, "planetara2"); 

	motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
	motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
	  
	  
	 
	 
	   
	   
	   telemetry.addData(">", "Press Play to start op mode");
	   telemetry.update();
	   
	   double leftCorner = 0;
	   
	   waitForStart();
	    sleep(1000);
	   initVuforia();
	   telemetry.addLine("vufo done");
	   sleep(1000);
	   initTfod();
	   telemetry.addLine("tfod done");
	   sleep(1000);
	   
	   if(tfod != null)
	   {
	   	tfod.activate();
	   	tfod.setZoom(1 , 16.0/9.0);
	   }
	   telemetry.addLine("gata.");
	   sleep(1000);
	   telemetry.update();
	   
	   
useEncoders();
resetEncoders();

	List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

while(updatedRecognitions == null && tfod.getUpdatedRecognitions()== null)
{
	updatedRecognitions = tfod.getUpdatedRecognitions();
}
   if(opModeIsActive())
	if(updatedRecognitions.size()>=1)
	{
		leftCorner = updatedRecognitions.get(0).getLeft();
	}
	else 
	{
		leftCorner = 200;
	}
	telemetry.addData("",leftCorner);
	
	if(isStopRequested()!=true){
	
if(updatedRecognitions.size()==1){
	
	if(leftCorner<200)
	{
		telemetry.addLine("CazJos");
    	telemetry.update();
		//CazJos();
	}
	else
	if(leftCorner>200)
	{
		telemetry.addLine("CazMijloc");
    	telemetry.update();
	//CazMijloc();
	}
}
else{
	telemetry.addLine("CazSus");
    	telemetry.update();
//	CazSus();
}

	}
    }
}







