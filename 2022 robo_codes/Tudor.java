// package Recruti;

// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// @Autonomous

// public class Tudor extends LinearOpMode
// {
//  private DcMotorEx motorDreaptaFata = null;
//  private DcMotorEx motorStangaFata = null;
//  private DcMotorEx motorDreaptaSpate = null;
//  private DcMotorEx motorStangaSpate = null;
//  private DcMotor hex = null;
//  private DcMotor hex1 = null;
//  private DcMotor motorManuta = null;
 
// public void runOpMode()
// { 
//  motorDreaptaFata=hardwareMap.get(DcMotorEx.class,"motorDreaptaFata");
//  motorDreaptaSpate=hardwareMap.get(DcMotorEx.class,"motorDreaptaSpate");
//  motorStangaFata=hardwareMap.get(DcMotorEx.class,"motorStangaFata");
//  motorStangaSpate=hardwareMap.get(DcMotorEx.class,"motorStangaSpate" );
 
// motorDreaptaFata.setDirection(DcMotor.Direction.REVERSE);
// motorDreaptaSpate.setDirection(DcMotor.Direction.REVERSE);

// motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// waitForStart();

// if (opModeIsActive())
// {
// motorDreaptaFata.setTargetPosition(1120);
// motorDreaptaSpate.setTargetPosition(1120);
// motorStangaSpate.setTargetPosition(1120);
// motorStangaFata.setTargetPosition(1120);

// motorDreaptaFata.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
// motorDreaptaSpate.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
// motorStangaFata.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
// motorStangaSpate.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

// motorDreaptaFata.setVelocity(300);
// motorDreaptaSpate.setVelocity(300);
// motorStangaFata.setVelocity(300);
// motorStangaSpate.setVelocity(300);

// while (motorDreaptaFata.isBusy()){}
// motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// motorDreaptaFata.setTargetPosition(-2240);
// motorDreaptaSpate.setTargetPosition(-2240);
// motorStangaSpate.setTargetPosition(2240);
// motorStangaFata.setTargetPosition(2240);
// motorDreaptaFata.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
// motorDreaptaSpate.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
// motorStangaFata.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
// motorStangaSpate.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
// motorDreaptaFata.setVelocity(1120);
// motorDreaptaSpate.setVelocity(1000);
// motorStangaFata.setVelocity(1120);
// motorStangaSpate.setVelocity(1120);


// while (motorDreaptaFata.isBusy()){}
// motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// }
// }
// }
