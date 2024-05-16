
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "!!TeleOp2023 startPos-centered drive")
public class DriverRelativeMovement extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;

    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    DcMotor mana = null;
    Servo wrist, hand = null;

    int x = 0;
    double c = 0;
    public void Hand(boolean y)
    {
        if(y)  hand.setPosition(0.6);
        if(!y) hand.setPosition(0.8);
    }
    public void Arm(int a, double b)
    {
        mana.setTargetPosition((int)(a));
        mana.setPower(b);
        mana.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    @Override
    public void runOpMode() {

        mana = hardwareMap.get(DcMotor.class, "Manuta");
        hand = hardwareMap.get(Servo.class, "sv0");
        wrist = hardwareMap.get(Servo.class, "sv1");


        mana.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mana.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setPosition(0.45);

        double driveTurn;

        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot;
        double gamepadDegree;
        double robotDegree;
        double movementDegree;
        double gamepadXControl;
        double gamepadYControl;

        frontLeft = hardwareMap.dcMotor.get("motorStangaFata");
        frontRight = hardwareMap.dcMotor.get("motorDreaptaFata");
        backRight = hardwareMap.dcMotor.get("motorDreaptaSpate");
        backLeft = hardwareMap.dcMotor.get("motorStangaSpate");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        while (opModeIsActive()) {
            driveTurn = gamepad1.right_stick_x;
// de rezolvat inacuratetea la grade gamepad
            gamepadXCoordinate = -gamepad1.left_stick_x; //this simply gives our x value relative to the driver
            gamepadYCoordinate = gamepad1.left_stick_y; //this simply gives our y vaue relative to the driver

            gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);

            gamepadDegree = Math.toDegrees(Math.atan2(gamepadYCoordinate, gamepadXCoordinate));
            robotDegree = getAngle();
            movementDegree = gamepadDegree - robotDegree;

            gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
            gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;

            frontRight.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            backRight.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            frontLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
            backLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);

            telemetry.addData("rogot degrees ", robotDegree);
            telemetry.addData("gamepad degree ", gamepadDegree);
            telemetry.addData("movement degree ", movementDegree);

            //Manual_Arm
            if(gamepad1.right_trigger > 0.95 && x > -1600)
            {
                x = x - 10;
                c = 0.99;
                if(mana.getCurrentPosition()<=-200)wrist.setPosition(0.55);
            }

            if(gamepad1.left_trigger > 0.95 && x < 0)
            {
                x = 0;
                c = 0;
                wrist.setPosition(0.45);
            }
            Arm(x, c);
            telemetry.addData("x-ul este ", x);
            telemetry.addData("c-ul este ", c);

            //Hold
            if(gamepad1.left_bumper) Hand(false);
            if(gamepad1.right_bumper) Hand(true);

            //Auto_Arm

            //Sus
            if(gamepad1.dpad_up)
            {
                x = -1300;
                c = 0.99;
                wrist.setPosition(0.55);
            }
            //Junk_2
            if(gamepad1.dpad_right)
            {
                x = -750;
                c = 0.99;
                wrist.setPosition(0.55);
            }
            //Junk_1
            if(gamepad1.dpad_left)
            {
                x = -450;
                c = 0.99;
                wrist.setPosition(0.55);
            }
            //Jos
            if(gamepad1.dpad_down)
            {
                x = 0;
                c = 0;
                wrist.setPosition(0.45);
            }
            //3 Stack
            if(gamepad1.x)
            {
                wrist.setPosition(0.55);
            }
            if(gamepad1.a)
            {
                wrist.setPosition(0.45);
            }
            if(gamepad1.y)
            {
                wrist.setPosition(0.85);
            }
            telemetry.update();
        }
    }

    void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });
    }

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}