// /* Copyright (c) 2019 FIRST. All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without modification,
//  * are permitted (subject to the limitations in the disclaimer below) provided that
//  * the following conditions are met:
//  *
//  * Redistributions of source code must retain the above copyright notice, this list
//  * of conditions and the following disclaimer.
//  *
//  * Redistributions in binary form must reproduce the above copyright notice, this
//  * list of conditions and the following disclaimer in the documentation and/or
//  * other materials provided with the distribution.
//  *
//  * Neither the name of FIRST nor the names of its contributors may be used to endorse or
//  * promote products derived from this software without specific prior written permission.
//  *
//  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//  * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */

// package Autonomie;

// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import java.util.List;
// import org.firstinspires.ftc.robotcore.external.ClassFactory;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
// import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
// import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

// /**
//  * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
//  * determine the position of the Ultimate Goal game elements.
//  *
//  * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
//  * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
//  *
//  * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
//  * is explained below.
//  */
// @Autonomous(name = "AutonomieRobot2021", group = "Concept")

// public class AutonomieRobot2021 extends LinearOpMode 
// {
//	private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//	private static final String LABEL_FIRST_ELEMENT = "Quad";
//	private static final String LABEL_SECOND_ELEMENT = "Single";
    
//	public String aux;
//	public int NumOfRings = -1; 
   
//	DcMotor motorStangaSpate;
//	DcMotor motorStangaFata;
//	DcMotor motorDreaptaSpate;
//	DcMotor motorDreaptaFata;
    
//	DcMotor hex;
//	DcMotor motorShooter1;
//	DcMotor motorShooter2;
    
//	Servo servoManuta1;
//	Servo servoManuta2;
    
//	BNO055IMU			imu;
//	Orientation		   lastAngles = new Orientation();
//	double			   globalAngle, correction;
    
//	static final int MOTOR_TICK_COUNT = 560;

//	/*
//	 * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
//	 * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
//	 * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
//	 * web site at https://developer.vuforia.com/license-manager.
//	 *
//	 * Vuforia license keys are always 380 characters long, and look as if they contain mostly
//	 * random data. As an example, here is a example of a fragment of a valid key:
//	 *	 ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
//	 * Once you've obtained a license key, copy the string from the Vuforia web site
//	 * and paste it in to your code on the next line, between the double quotes.
//	 */
//	private static final String VUFORIA_KEY =
//		   "Ac20cOT/////AAABmbPVklXrQ0v2oizK4TTtbk5vs2iYKT+A3BuAPhSYgxQcnilhP8FqnhvfhL3YZ/HZKBNAyD13HQhsk/9WBEXi71oXSawGov3CwwaPItHLfzqvaRbPNEtUmQX5Y/Pzu3md8rO3aWOCDL8QhJolPUfv2ZqxNR+C1zxNMYYON2+el5HYX2Rg8gPT0qvTVKlWpXu2GofPmAZl+tBsoOBLo/8JCEIjTwk05awTubK8nPOkpkKd95uiHrwG5vSt7RTJtHasqw0sH+HipeD7b1DEp4otpCgE+LtiAfk2hXeeitV4uxLSXdARtsuUshaagvuI47vb5MZpRmBilcekXtm76us/oKt7NLI6WjPce3CZfZthdzys";

//	/**
//	 * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
//	 * localization engine.
//	 */
//	private VuforiaLocalizer vuforia;

//	/**
//	 * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
//	 * Detection engine.
//	 */
//	private TFObjectDetector tfod;

//	@Override
//	public void runOpMode(){
//	    // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
//	    // first.
//	    initVuforia();
//	    initTfod();

//	    /**
//		* Activate TensorFlow Object Detection before we wait for the start command.
//		* Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
//		**/
//	    if (tfod != null) {
//		   tfod.activate();

//		   // The TensorFlow software will scale the input images from the camera to a lower resolution.
//		   // This can result in lower detection accuracy at longer distances (> 55cm or 22").
//		   // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
//		   // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
//		   // should be set to the value of the images used to create the TensorFlow Object Detection model
//		   // (typically 1.78 or 16/9).

//		   // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
//		   //tfod.setZoom(2.5, 1.78);
//	    }

//	    /** Wait for the game to begin */
//	    telemetry.addData("Camera", "Pregatita. Asteptati calibrarea giroscopului!");
//	    telemetry.update();
	   
//	    servoManuta1 = hardwareMap.get(Servo.class, "servoManuta1");
//	    servoManuta2 = hardwareMap.get(Servo.class, "servoManuta2");
	   
//	    motorDreaptaFata  = hardwareMap.get(DcMotor.class, "motorDreaptaFata");
//	    motorDreaptaSpate = hardwareMap.get(DcMotor.class, "motorDreaptaSpate");
//	    motorStangaFata   = hardwareMap.get(DcMotor.class, "motorStangaFata");
//	    motorStangaSpate  = hardwareMap.get(DcMotor.class, "motorStangaSpate");
	   
//	    hex = hardwareMap.get(DcMotor.class, "hex");
//	    motorShooter1 = hardwareMap.get(DcMotor.class, "motorShooter1");
//	    motorShooter2 = hardwareMap.get(DcMotor.class, "motorShooter2");
	   
//	    motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
//	    motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);
	   
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   
//	    motorShooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorShooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	   
//	    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
	   
//	    parameters.mode			 = BNO055IMU.SensorMode.IMU;
//	    parameters.angleUnit		 = BNO055IMU.AngleUnit.DEGREES;
//	    parameters.accelUnit		 = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//	    parameters.loggingEnabled	 = false;

//	    imu = hardwareMap.get(BNO055IMU.class, "imu");
	   
//	    imu.initialize(parameters);

//	    telemetry.addData("Giroscop", "se calibreaza...");
//	    telemetry.update();

//	    while (!isStopRequested() && !imu.isGyroCalibrated())
//	    {
//		   sleep(50);
//		   idle();
//	    }
	   
//	    telemetry.addData("Giroscop", "calibrat. Puteti incepe autonomia!");
//	    telemetry.update();
	   
//	    waitForStart();
	   
//	    if (opModeIsActive()) {
//			  if (tfod != null) {
//				 List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//				 if (updatedRecognitions != null) {
//				   telemetry.addData("# Obiecte detectate", updatedRecognitions.size());
//				   int i = 0;
//				   for (Recognition recognition : updatedRecognitions) {
//					telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//					aux = recognition.getLabel();
//					telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//						   recognition.getLeft(), recognition.getTop());
//					telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//						   recognition.getRight(), recognition.getBottom());
//				   }
				 
//					if(NumOfRings == -1)
//					{
//					    if(aux == "Quad") NumOfRings = 4;
//					    else if(aux == "Single") NumOfRings = 1;
//					    else if(updatedRecognitions.size() == 0) NumOfRings = 0;
					   
//					    telemetry.update();
//					    telemetry.addData("Cercuri", NumOfRings);
					   
//					    if (NumOfRings == 0)
//						{
//							servoManuta2.setPosition(0.1);
//							sleep(500);
//							driveForwardWithShroom(152);
//							servoManuta1.setPosition(0.55);
//							sleep(750);
//							servoManuta2.setPosition(0.5);
//							servoManuta1.setPosition(0.1);
//							rotateNoShroom(164);
//							servoManuta1.setPosition(0.55);
//							driveForwardNoShroom(103);
//							servoManuta2.setPosition(0.1);
//							sleep(750);
//							servoManuta1.setPosition(0.1);
//							rotateWithShroom(-170);
//							driveForwardWithShroom(100);
//							servoManuta1.setPosition(0.55);
//							sleep(750);
//							servoManuta2.setPosition(0.5);
//							servoManuta1.setPosition(0.1);
//							rotateNoShroom(12);
//							driveBackwardsNoShroom(32);
//							Shoot(0);
//							driveForwardNoShroom(60);
//						}
					   
//						else if (NumOfRings == 1)
//						{
//						    servoManuta2.setPosition(0.1);
//						    driveForwardWithShroom(198);
//						    rotateWithShroom(50);
//						    servoManuta1.setPosition(0.55);
//						    sleep(400);
//						    servoManuta2.setPosition(0.5);
//						    sleep(250);
//						    servoManuta1.setPosition(0.1);
//						    sleep(250);
//						    rotateNoShroom(119);
//						    servoManuta1.setPosition(0.55);
//						    driveForwardNoShroom(148);
//						    servoManuta2.setPosition(0.1);
//						    sleep(200);
//						    servoManuta1.setPosition(0.1);
//						    sleep(150);
//						    rotateWithShroom(-154);
//						    driveForwardWithShroom(150);
//						    servoManuta1.setPosition(0.55);
//						    sleep(350);
//						    servoManuta2.setPosition(0.5);
//						    sleep(200);
//						    servoManuta1.setPosition(0.1);
//						    sleep(150);
//						    driveBackwardsNoShroom(80);
//						    rotateNoShroom(-13);
//						    Shoot(1);
//						    driveForwardNoShroom(70);
//						}
					   
//					    else if (NumOfRings == 4)
//					    {
//						   servoManuta2.setPosition(0.1);
//						   sleep(500);
//						   driveForwardWithShroomCase3(292);
//						   servoManuta1.setPosition(0.6);
//						   sleep(500);
//						   servoManuta2.setPosition(0.5);
//						   sleep(1000);
//						   servoManuta1.setPosition(0);
//						   rotateNoShroom(175);
//						   servoManuta1.setPosition(0.55);
//						   driveForwardNoShroomCase3(227);
//						   servoManuta2.setPosition(0.1);
//						   sleep(200);
//						   servoManuta1.setPosition(0.1);
//						   rotateWithShroom(-175);
//						   driveForwardWithShroomCase3(205);
//						   servoManuta1.setPosition(0.55);
//						   sleep(1000);
//						   servoManuta2.setPosition(0.5);
//						   sleep(100);
//						   servoManuta1.setPosition(0.1);
//						   resetAngle();
//						   driveBackwardsNoShroomCase3(115);
//						   rotateNoShroom(7);
//						   Shoot(4);
//						   driveForwardNoShroomCase3(30);
//					    }
//					}
//				 }
//			  }
//	    }
	   
//	    if (tfod != null) {
//		   tfod.shutdown();
//	    }
//	}

//	/**
//	 * Initialize the Vuforia localization engine.
//	 */
//	private void initVuforia() {
//	    /*
//		* Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//		*/
//	    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

//	    parameters.vuforiaLicenseKey = VUFORIA_KEY;
//	    parameters.cameraName = hardwareMap.get(WebcamName.class, "vod");

//	    //  Instantiate the Vuforia engine
//	    vuforia = ClassFactory.getInstance().createVuforia(parameters);

//	    // Loading trackables is not necessary for the TensorFlow Object Detection engine.
//	}

//	private void initTfod() {
//	    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//		   "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//	    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//	  tfodParameters.minResultConfidence = 0.6f;
//	  tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//	  tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
//	}
    
//	private void resetAngle()
//	{
//	    lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//	    globalAngle = 0;
//	}
    
//	private double getAngle()
//	{
//	    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//	    double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

//	    if (deltaAngle < -180)
//		   deltaAngle += 360;
//	    else if (deltaAngle > 180)
//		   deltaAngle -= 360;

//	    globalAngle += deltaAngle;

//	    lastAngles = angles;

//	    return globalAngle;
//	}
    
    
//	private double checkDirection(int caz)
//	{
//	    double correction, angle, gain = 0;
	   
//	    if (caz == 1) // robotul merge fata - spate fara ciuperca in mana | driveForwardNoShroom(distance) | driveBackwardsNoShroom(distance)
//	    {
//		   //gain = 0.03;
//		   gain = 0.03;
//	    }
	   
//	    else if (caz == 2) // robotul se roteste fara ciuperca in mana | rotateNoShroom(degrees) |
//	    {
//		   //gain = 0.29;
//		   gain = 0.29;
//	    }
	   
//	    else if (caz == 3) // robotul merge in fata cu ciuperca in mana | driveForwardWithShroom(distance) |
//	    {
//		   gain = 0.01;
//	    }
	   
//	    else if (caz == 4) // robotul se roteste cu ciuperca in mana | rotateWithShroom(degrees) |
//	    {
//		   //gain = 0.6;
//		   gain = 0.4;
//	    }

//	    angle = getAngle();

//	    if (angle == 0)
//		   correction = 0;		   
//	    else
//		   correction = -angle;	   

//	    correction = correction * gain;

//	    return correction;
//	}
    
//	private void rotateNoShroom(int degrees)
//	{
//	    resetAngle();
    
//	    motorDreaptaFata.setTargetPosition(degrees * 9);
//	    motorDreaptaSpate.setTargetPosition(degrees * 9);
//	    motorStangaFata.setTargetPosition(-degrees * 9);
//	    motorStangaSpate.setTargetPosition(-degrees * 9);
		  
//	    motorDreaptaFata.setPower(1);
//	    motorDreaptaSpate.setPower(1);
//	    motorStangaSpate.setPower(1);
//	    motorStangaFata.setPower(1);

//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		  
//	    while (motorStangaFata.isBusy() || motorStangaSpate.isBusy() || motorDreaptaFata.isBusy() || motorDreaptaSpate.isBusy())
//	    {
//		   correction = checkDirection(2);
    
//		   telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//		   telemetry.addData("Global Heading: ", globalAngle);
//		   telemetry.addData("Correction: ", correction);
//		   telemetry.update();
	   
//		   motorDreaptaFata.setPower(1 + correction);
//		   motorDreaptaSpate.setPower(1 + correction);
//		   motorStangaSpate.setPower(1 - correction);
//		   motorStangaFata.setPower(1 - correction);
//	    }
		   
//	    while (getAngle() < degrees - 0.5  || getAngle() > degrees + 0.5)
//	    {
//		   if (getAngle() > degrees)
//		   {
//			  correction = checkDirection(2);
    
//			  telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//			  telemetry.addData("Global Heading: ", globalAngle);
//			  telemetry.addData("Correction: ", correction);
//			  telemetry.update();
	   
//			  motorDreaptaFata.setPower(1 - correction);
//			  motorDreaptaSpate.setPower(1 - correction);
//			  motorStangaSpate.setPower(1 + correction);
//			  motorStangaFata.setPower(1 + correction);
				
//			  motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				
//			  motorDreaptaFata.setTargetPosition(-40);
//			  motorDreaptaSpate.setTargetPosition(-40);
//			  motorStangaFata.setTargetPosition(40);
//			  motorStangaSpate.setTargetPosition(40);
				
//			  motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		   }
		  
//		   else if (getAngle() < degrees)
//		   {
//			  correction = checkDirection(2);
    
//			  telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//			  telemetry.addData("Global Heading: ", globalAngle);
//			  telemetry.addData("Correction: ", correction);
//			  telemetry.update();
	   
//			  motorDreaptaFata.setPower(1 + correction);
//			  motorDreaptaSpate.setPower(1 + correction);
//			  motorStangaSpate.setPower(1 - correction);
//			  motorStangaFata.setPower(1 - correction);
				
//			  motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				
//			  motorDreaptaFata.setTargetPosition(40);
//			  motorDreaptaSpate.setTargetPosition(40);
//			  motorStangaFata.setTargetPosition(-40);
//			  motorStangaSpate.setTargetPosition(-40);
				
//			  motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		   }
//	    }
	   
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
//	    motorDreaptaFata.setPower(0);
//	    motorDreaptaSpate.setPower(0);
//	    motorStangaSpate.setPower(0);
//	    motorStangaFata.setPower(0); 
	   
//	    resetAngle();
	   
//	}
    
//	private void rotateWithShroom(int degrees)
//	{
//	    resetAngle();
    
//	    motorDreaptaFata.setTargetPosition(degrees * 9);
//	    motorDreaptaSpate.setTargetPosition(degrees * 9);
//	    motorStangaFata.setTargetPosition(-degrees * 9);
//	    motorStangaSpate.setTargetPosition(-degrees * 9);
		  
//	    motorDreaptaFata.setPower(1);
//	    motorDreaptaSpate.setPower(1);
//	    motorStangaSpate.setPower(1);
//	    motorStangaFata.setPower(1);

//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		  
//	    while(motorStangaFata.isBusy() || motorStangaSpate.isBusy() || motorDreaptaFata.isBusy() || motorDreaptaSpate.isBusy())
//	    {
//		   correction = checkDirection(2);
    
//		   telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//		   telemetry.addData("Global Heading: ", globalAngle);
//		   telemetry.addData("Correction: ", correction);
//		   telemetry.update();
	   
//		   motorDreaptaFata.setPower(1 + correction);
//		   motorDreaptaSpate.setPower(1 + correction);
//		   motorStangaSpate.setPower(1 - correction);
//		   motorStangaFata.setPower(1 - correction);
//	    }
		   
//	    while (getAngle() < degrees - 0.5  || getAngle() > degrees + 0.5)
//	    {
//		   if (getAngle() > degrees)
//		   {
//			  correction = checkDirection(2);
    
//			  telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//			  telemetry.addData("Global Heading: ", globalAngle);
//			  telemetry.addData("Correction: ", correction);
//			  telemetry.update();
	   
//			  motorDreaptaFata.setPower(1 - correction);
//			  motorDreaptaSpate.setPower(1 - correction);
//			  motorStangaSpate.setPower(1 + correction);
//			  motorStangaFata.setPower(1 + correction);
				
//			  motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				
//			  motorDreaptaFata.setTargetPosition(-40);
//			  motorDreaptaSpate.setTargetPosition(-40);
//			  motorStangaFata.setTargetPosition(40);
//			  motorStangaSpate.setTargetPosition(40);
				
//			  motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		   }
		  
//		   else if (getAngle() < degrees)
//		   {
//			  correction = checkDirection(2);
    
//			  telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//			  telemetry.addData("Global Heading: ", globalAngle);
//			  telemetry.addData("Correction: ", correction);
//			  telemetry.update();
	   
//			  motorDreaptaFata.setPower(1 + correction);
//			  motorDreaptaSpate.setPower(1 + correction);
//			  motorStangaSpate.setPower(1 - correction);
//			  motorStangaFata.setPower(1 - correction);
				
//			  motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			  motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				
//			  motorDreaptaFata.setTargetPosition(40);
//			  motorDreaptaSpate.setTargetPosition(40);
//			  motorStangaFata.setTargetPosition(-40);
//			  motorStangaSpate.setTargetPosition(-40);
				
//			  motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			  motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		   }
//	    }
	   
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
//	    motorDreaptaFata.setPower(0);
//	    motorDreaptaSpate.setPower(0);
//	    motorStangaSpate.setPower(0);
//	    motorStangaFata.setPower(0); 
	   
//	    resetAngle();
//	    }
	   
    
    
//	private void driveForwardNoShroom(double distance)
//	{ 
//	    double diameter = 10;
//	    double circumference = 3.14 * diameter;
//	    double rotationsNeeded = distance / circumference;
//	    int encoderTargetPosition = (int)(rotationsNeeded * MOTOR_TICK_COUNT);

//	    motorDreaptaFata.setTargetPosition(encoderTargetPosition);
//	    motorDreaptaSpate.setTargetPosition(encoderTargetPosition);
//	    motorStangaFata.setTargetPosition(encoderTargetPosition);
//	    motorStangaSpate.setTargetPosition(encoderTargetPosition);

//	    motorDreaptaFata.setPower(0.5);
//	    motorDreaptaSpate.setPower(0.5);
//	    motorStangaSpate.setPower(0.5);
//	    motorStangaFata.setPower(0.5);

//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//	    while (motorDreaptaFata.isBusy() || motorDreaptaSpate.isBusy() || motorStangaSpate.isBusy() || motorStangaFata.isBusy()) 
//	    {
//		   correction = checkDirection(1);

//		   telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//		   telemetry.addData("Global Heading: ", globalAngle);
//		   telemetry.addData("Correction: ", correction);
//		   telemetry.update();

//		   motorDreaptaFata.setPower(0.5 + correction);
//		   motorDreaptaSpate.setPower(0.5 + correction);
//		   motorStangaSpate.setPower(0.5 - correction);
//		   motorStangaFata.setPower(0.5 - correction);
//	    }
	   
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
//	    motorDreaptaFata.setPower(0);
//	    motorDreaptaSpate.setPower(0);
//	    motorStangaSpate.setPower(0);
//	    motorStangaFata.setPower(0); 
//	}
    
//	private void driveForwardWithShroom(double distance)
//	{ 
//	    double diameter = 10;
//	    double circumference = 3.14 * diameter;
//	    double rotationsNeeded = distance / circumference;
//	    int encoderTargetPosition = (int)(rotationsNeeded * MOTOR_TICK_COUNT);

//	    motorDreaptaFata.setTargetPosition(encoderTargetPosition);
//	    motorDreaptaSpate.setTargetPosition(encoderTargetPosition);
//	    motorStangaFata.setTargetPosition(encoderTargetPosition);
//	    motorStangaSpate.setTargetPosition(encoderTargetPosition);

//	    motorDreaptaFata.setPower(0.5);
//	    motorDreaptaSpate.setPower(0.5);
//	    motorStangaSpate.setPower(0.5);
//	    motorStangaFata.setPower(0.5);

//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//	    while (motorDreaptaFata.isBusy() || motorDreaptaSpate.isBusy() || motorStangaSpate.isBusy() || motorStangaFata.isBusy()) 
//	    {
//		   correction = checkDirection(3);

//		   telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//		   telemetry.addData("Global Heading: ", globalAngle);
//		   telemetry.addData("Correction: ", correction);
//		   telemetry.update();

//		   motorDreaptaFata.setPower(0.5 + correction);
//		   motorDreaptaSpate.setPower(0.5 + correction);
//		   motorStangaSpate.setPower(0.5 - correction);
//		   motorStangaFata.setPower(0.5 - correction);
//	    }
	   
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
//	    motorDreaptaFata.setPower(0);
//	    motorDreaptaSpate.setPower(0);
//	    motorStangaSpate.setPower(0);
//	    motorStangaFata.setPower(0); 
//	}
    
//	private void driveBackwardsNoShroom(double distance)
//	{ 
//	    double diameter = 10;
//	    double circumference = 3.14 * diameter;
//	    double rotationsNeeded = distance / circumference;
//	    int encoderTargetPosition = (int)(rotationsNeeded * MOTOR_TICK_COUNT);

//	    motorDreaptaFata.setTargetPosition(-encoderTargetPosition);
//	    motorDreaptaSpate.setTargetPosition(-encoderTargetPosition);
//	    motorStangaFata.setTargetPosition(-encoderTargetPosition);
//	    motorStangaSpate.setTargetPosition(-encoderTargetPosition);

//	    motorDreaptaFata.setPower(0.5);
//	    motorDreaptaSpate.setPower(0.5);
//	    motorStangaSpate.setPower(0.5);
//	    motorStangaFata.setPower(0.5);

//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//	    while (motorDreaptaFata.isBusy() || motorDreaptaSpate.isBusy() || motorStangaSpate.isBusy() || motorStangaFata.isBusy()) 
//	    {
//		   correction = checkDirection(1);

//		   telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//		   telemetry.addData("Global Heading: ", globalAngle);
//		   telemetry.addData("Correction: ", correction);
//		   telemetry.update();

//		   motorDreaptaFata.setPower(0.5 - correction);
//		   motorDreaptaSpate.setPower(0.5 - correction);
//		   motorStangaSpate.setPower(0.5 + correction);
//		   motorStangaFata.setPower(0.5 + correction);
//	    }
	   
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
//	    motorDreaptaFata.setPower(0);
//	    motorDreaptaSpate.setPower(0);
//	    motorStangaSpate.setPower(0);
//	    motorStangaFata.setPower(0); 
//	}
    
//	private void Shoot(int caz)
//	{
	   
//	    if (caz == 0)
//	    {
//		   motorShooter1.setTargetPosition(100000);
//		   motorShooter2.setTargetPosition(-100000);
		  
//		   motorShooter1.setPower(0.50);
//		   motorShooter2.setPower(0.50);
		  
//		   motorShooter1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		   motorShooter2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		  
//		   sleep(1000);
		  
//		   hex.setPower(0.760);
//		   sleep(600);
//		   hex.setPower(0);
		  
//		   hex.setPower(1);
//		   sleep(600);
//		   hex.setPower(0);
		  
//		   hex.setPower(1);
//		   sleep(600);
		  
//		   // sleep(1500);
		  
//		   motorShooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		   motorShooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
//		   motorShooter1.setPower(0);
//		   motorShooter2.setPower(0);
//		   hex.setPower(0);
//	    }
	   
//	    else if (caz == 1)
//	    {
//		   motorShooter1.setTargetPosition(100000);
//		   motorShooter2.setTargetPosition(-100000);
		  
//		   motorShooter1.setPower(0.49);
//		   motorShooter2.setPower(0.49);
		  
//		   motorShooter1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		   motorShooter2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		  
//		   sleep(1000);
		  
//		   //hex.setPower(1);
//		   //sleep(1500);
//		   hex.setPower(0.760);
//		   sleep(400);
//		   hex.setPower(0);
		  
//		   hex.setPower(1);
//		   sleep(500);
//		   hex.setPower(0);
		  
//		   hex.setPower(1);
//		   sleep(400);
//		   motorShooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		   motorShooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
//		   motorShooter1.setPower(0);
//		   motorShooter2.setPower(0);
//		   hex.setPower(0);
//	    }
	   
//	    else if (caz == 4)
//	    {
//		   motorShooter1.setTargetPosition(100000);
//		   motorShooter2.setTargetPosition(-100000);
		  
//		   motorShooter1.setPower(0.5);
//		   motorShooter2.setPower(0.5);
		  
//		   motorShooter1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		   motorShooter2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		  
//		   sleep(1000);
		  
//		   //hex.setPower(1);
//		   //sleep(1500);
		  
//		   hex.setPower(0.760);
//		   sleep(600);
//		   hex.setPower(0);
		  
//		   hex.setPower(1);
//		   sleep(600);
//		   hex.setPower(0);
		  
//		   hex.setPower(1);
//		   sleep(600);
		  
//		   motorShooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		   motorShooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
//		   motorShooter1.setPower(0);
//		   motorShooter2.setPower(0);
//		   hex.setPower(0);
//	    }
//	}
    
    
//	//CASE 3 MOVING FUNCTIONS
    
    
    
//	private void driveForwardNoShroomCase3(double distance)
//	{ 
//	    double diameter = 10;
//	    double circumference = 3.14 * diameter;
//	    double rotationsNeeded = distance / circumference;
//	    int encoderTargetPosition = (int)(rotationsNeeded * MOTOR_TICK_COUNT);

//	    motorDreaptaFata.setTargetPosition(encoderTargetPosition);
//	    motorDreaptaSpate.setTargetPosition(encoderTargetPosition);
//	    motorStangaFata.setTargetPosition(encoderTargetPosition);
//	    motorStangaSpate.setTargetPosition(encoderTargetPosition);

//	    motorDreaptaFata.setPower(0.7);
//	    motorDreaptaSpate.setPower(0.7);
//	    motorStangaSpate.setPower(0.7);
//	    motorStangaFata.setPower(0.7);

//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//	    while (motorDreaptaFata.isBusy() || motorDreaptaSpate.isBusy() || motorStangaSpate.isBusy() || motorStangaFata.isBusy()) 
//	    {
//		   correction = checkDirection(1);

//		   telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//		   telemetry.addData("Global Heading: ", globalAngle);
//		   telemetry.addData("Correction: ", correction);
//		   telemetry.update();

//		   motorDreaptaFata.setPower(0.7 + correction);
//		   motorDreaptaSpate.setPower(0.7 + correction);
//		   motorStangaSpate.setPower(0.7 - correction);
//		   motorStangaFata.setPower(0.7 - correction);
//	    }
	   
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
//	    motorDreaptaFata.setPower(0);
//	    motorDreaptaSpate.setPower(0);
//	    motorStangaSpate.setPower(0);
//	    motorStangaFata.setPower(0); 
//	}
    
//	private void driveForwardWithShroomCase3(double distance)
//	{ 
//	    double diameter = 10;
//	    double circumference = 3.14 * diameter;
//	    double rotationsNeeded = distance / circumference;
//	    int encoderTargetPosition = (int)(rotationsNeeded * MOTOR_TICK_COUNT);

//	    motorDreaptaFata.setTargetPosition(encoderTargetPosition);
//	    motorDreaptaSpate.setTargetPosition(encoderTargetPosition);
//	    motorStangaFata.setTargetPosition(encoderTargetPosition);
//	    motorStangaSpate.setTargetPosition(encoderTargetPosition);

//	    motorDreaptaFata.setPower(0.7);
//	    motorDreaptaSpate.setPower(0.7);
//	    motorStangaSpate.setPower(0.7);
//	    motorStangaFata.setPower(0.7);

//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//	    while (motorDreaptaFata.isBusy() || motorDreaptaSpate.isBusy() || motorStangaSpate.isBusy() || motorStangaFata.isBusy()) 
//	    {
//		   correction = checkDirection(3);

//		   telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//		   telemetry.addData("Global Heading: ", globalAngle);
//		   telemetry.addData("Correction: ", correction);
//		   telemetry.update();

//		   motorDreaptaFata.setPower(0.7 + correction);
//		   motorDreaptaSpate.setPower(0.7 + correction);
//		   motorStangaSpate.setPower(0.7 - correction);
//		   motorStangaFata.setPower(0.7 - correction);
//	    }
	   
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
//	    motorDreaptaFata.setPower(0);
//	    motorDreaptaSpate.setPower(0);
//	    motorStangaSpate.setPower(0);
//	    motorStangaFata.setPower(0); 
//	}
    
//	private void driveBackwardsNoShroomCase3(double distance)
//	{ 
//	    double diameter = 10;
//	    double circumference = 3.14 * diameter;
//	    double rotationsNeeded = distance / circumference;
//	    int encoderTargetPosition = (int)(rotationsNeeded * MOTOR_TICK_COUNT);

//	    motorDreaptaFata.setTargetPosition(-encoderTargetPosition);
//	    motorDreaptaSpate.setTargetPosition(-encoderTargetPosition);
//	    motorStangaFata.setTargetPosition(-encoderTargetPosition);
//	    motorStangaSpate.setTargetPosition(-encoderTargetPosition);

//	    motorDreaptaFata.setPower(0.7);
//	    motorDreaptaSpate.setPower(0.7);
//	    motorStangaSpate.setPower(0.7);
//	    motorStangaFata.setPower(0.7);

//	    motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//	    motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//	    while (motorDreaptaFata.isBusy() || motorDreaptaSpate.isBusy() || motorStangaSpate.isBusy() || motorStangaFata.isBusy()) 
//	    {
//		   correction = checkDirection(1);

//		   telemetry.addData("IMU heading: ", lastAngles.firstAngle);
//		   telemetry.addData("Global Heading: ", globalAngle);
//		   telemetry.addData("Correction: ", correction);
//		   telemetry.update();

//		   motorDreaptaFata.setPower(0.7 - correction);
//		   motorDreaptaSpate.setPower(0.7 - correction);
//		   motorStangaSpate.setPower(0.7 + correction);
//		   motorStangaFata.setPower(0.7 + correction);
//	    }
	   
//	    motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//	    motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		  
//	    motorDreaptaFata.setPower(0);
//	    motorDreaptaSpate.setPower(0);
//	    motorStangaSpate.setPower(0);
//	    motorStangaFata.setPower(0); 
//	}
// }
