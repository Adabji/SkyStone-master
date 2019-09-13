package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoByIMU", group = "Autonomous")
public class AutoByIMU extends LinearOpMode {

    private static DcMotor leftFrontWheel;
    private static DcMotor leftBackWheel;
    private static DcMotor rightFrontWheel;
    private static DcMotor rightBackWheel;
    // private static double Theta;
    // private static double r;
    // private static double heading;
    // Double Deg = Math.PI;
    private BNO055IMU imu;
    private double correction, globalAngle;
    private Orientation angles;
    private int rotateTolerance = 5;

    Orientation lastAngles = new Orientation();

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontWheel = hardwareMap.dcMotor.get("left front");
        leftBackWheel = hardwareMap.dcMotor.get("left back");
        rightFrontWheel = hardwareMap.dcMotor.get("right front");
        rightBackWheel = hardwareMap.dcMotor.get("right back");

        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        while(opModeIsActive()) {
            moveToLoc(90, 500, 0.1);
            telemetry.addData("Heading:",getAbsoluteHeading());
        }
    }

    private void moveToLoc(int head, int distance, double power) {
        rotate(head, power);
        // move(distance, power);
    }

    private void rotate(int head, double power)
    {
        telemetry.addData("Rotating...", head);
        telemetry.update();

        double leftPower, rightPower;
        double rotateAngle;
        if(getAbsoluteHeading() < 0) rotateAngle = Math.abs(getAbsoluteHeading());
        else if(getAbsoluteHeading() > 0) rotateAngle = getAbsoluteHeading() + 180;
        else return;

        // Resume working here.



        // set power to rotate.
        leftFrontWheel.setPower(power);
        leftBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
        rightBackWheel.setPower(power);

        /* rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}*/

        if(getAbsoluteHeading() >  head - rotateTolerance && getAbsoluteHeading() < head + rotateTolerance){
            // turn the motors off.
            telemetry.addData("Motor stopping", "stopping");
            telemetry.update();
            leftFrontWheel.setPower(0);
            leftBackWheel.setPower(0);
            rightFrontWheel.setPower(0);
            rightBackWheel.setPower(0);
        }

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        // resetAngle();
    }

    private void move(int distance, double power) {

        telemetry.addData("moving", distance);
        // correction = checkDirection();

        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontWheel.setTargetPosition(distance);
        leftBackWheel.setTargetPosition(distance);
        rightFrontWheel.setTargetPosition(distance);
        rightBackWheel.setTargetPosition(distance);

        leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontWheel.setPower(power);
        leftBackWheel.setPower(power);
        rightFrontWheel.setPower(power);
        rightBackWheel.setPower(power);

        while (leftFrontWheel.isBusy() && leftBackWheel.isBusy() && rightFrontWheel.isBusy() && rightBackWheel.isBusy()){ }

        leftFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightFrontWheel.setPower(0);
        rightBackWheel.setPower(0);

        leftFrontWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public double getAbsoluteHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }

    private Double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.valueOf(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
