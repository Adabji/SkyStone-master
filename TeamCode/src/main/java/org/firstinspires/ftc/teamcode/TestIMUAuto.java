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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TestIMUAuto", group = "Autonomous")
public class TestIMUAuto extends LinearOpMode {

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

    Orientation lastAngles = new Orientation();

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontWheel = hardwareMap.dcMotor.get("left front");
        leftBackWheel = hardwareMap.dcMotor.get("left back");
        rightFrontWheel = hardwareMap.dcMotor.get("right front");
        rightBackWheel = hardwareMap.dcMotor.get("right back");

        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        if(opModeIsActive()) {
            moveToLoc(90, 500, 90, 0.1, 1);
            // moveToLoc(45, 300, 45, 0.5, 2);
            // moveToLoc(-90, 500, 60, 0.8, 1);
        }
    }

    private void moveToLoc(int firstHeading, int distance, int secondHeading, double rotatePower, double movePower) {
        head(firstHeading, rotatePower);
        move(distance, movePower);
        head(secondHeading, rotatePower);
    }

    private void head(int heading, double rPower) {
        telemetry.addData("Current Heading: ", getAbsoluteHeading());
        telemetry.update();

        double leftPower, rightPower;

        if(heading < getAbsoluteHeading()) {
            // to turn right
            telemetry.addData("turning right: ", heading);
            telemetry.update();
            leftPower = rPower;
            rightPower = -rPower;
        } else if(heading > getAbsoluteHeading()) {
            // to turn left
            telemetry.addData("turning left: ", heading);
            telemetry.update();
            leftPower = -rPower;
            rightPower = rPower;
        } else return;

        // set power to rotate.
        leftFrontWheel.setPower(leftPower);
        leftBackWheel.setPower(leftPower);
        rightFrontWheel.setPower(rightPower);
        rightBackWheel.setPower(rightPower);

        /* rotate until turn is completed.
        if (rAngle < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > rAngle) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < rAngle) {}*/

        while (opModeIsActive() && getAbsoluteHeading() != heading) {}

        // turn the motors off.
        leftFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightFrontWheel.setPower(0);
        rightBackWheel.setPower(0);

        sleep(1000);
    }

    private void move(int distance, double mPower) {
        telemetry.addData("moving: ", distance);
        telemetry.update();
        
        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontWheel.setTargetPosition(distance);
        leftBackWheel.setTargetPosition(distance);
        rightFrontWheel.setTargetPosition(distance);
        rightBackWheel.setTargetPosition(distance);

        leftFrontWheel.setPower(mPower);
        leftBackWheel.setPower(mPower);
        rightFrontWheel.setPower(mPower);
        rightBackWheel.setPower(mPower);

        while (leftFrontWheel.isBusy() && leftBackWheel.isBusy() && rightFrontWheel.isBusy() && rightBackWheel.isBusy()) {}

        leftFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightFrontWheel.setPower(0);
        rightBackWheel.setPower(0);

        leftFrontWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /*private void moveToLoc(int deg, int distance, int power) {
        rotate(deg, power);
        move(distance, power);
    }*/



    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param //degrees Degrees to turn, + is left - is right

    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        leftFrontWheel.setPower(leftPower);
        leftBackWheel.setPower(leftPower);
        rightFrontWheel.setPower(rightPower);
        rightBackWheel.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        leftFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightFrontWheel.setPower(0);
        rightBackWheel.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void move(int distance, int power) {

        correction = checkDirection();

        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontWheel.setTargetPosition(distance);
        leftBackWheel.setTargetPosition(distance);
        rightFrontWheel.setTargetPosition(-distance);
        rightBackWheel.setTargetPosition(-distance);

        leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontWheel.setPower(power - correction);
        leftBackWheel.setPower(power - correction);
        rightFrontWheel.setPower(power + correction);
        rightBackWheel.setPower(power + correction);

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

    private double checkDirection(){
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }*/

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

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }*/

}
