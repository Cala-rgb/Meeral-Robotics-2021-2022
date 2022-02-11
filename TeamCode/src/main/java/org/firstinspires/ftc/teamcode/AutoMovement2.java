package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutoMovement2 {
    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private BNO055IMU imu;
    private LinearOpMode lom;
    private double lastAngle = 0.0;
    int valpatrat = 1000;
    double minPower;
    VoltageSensor vs;

    public enum Directions {FORWARD, BACKWARD, LEFT, RIGHT, ROTATE_RIGHT, ROTATE_LEFT};
    public AutoMovement2 (LinearOpMode lom, BNO055IMU imu, DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, VoltageSensor vs) {
        this.lom = lom;
        this.imu = imu;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.vs = vs;
        this.minPower = 0.3 * (11 / vs.getVoltage());
    }

    private double getCurrentPower(int currentTick, int encoderStopAccelerate, int encoderStartBrake, int encoderTarget, double maxPower) {
        if (currentTick >= encoderTarget) {
            return 0;
        }
        if (currentTick == 0) {
            return minPower;
        }
        if (currentTick <= encoderStopAccelerate) {
            return Math.max(minPower, ((currentTick * maxPower) / encoderStopAccelerate));
        }
        if (currentTick < encoderStartBrake) {
            return maxPower;
        }
        return Math.max(minPower, ((maxPower * (encoderTarget - currentTick)) / (encoderTarget - encoderStartBrake)));
    }

    private double getAngle() {
        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle.firstAngle;
    }

    private double getEncoderAverage() {
        return (frontRight.getCurrentPosition() + frontLeft.getCurrentPosition() + backRight.getCurrentPosition() + backLeft.getCurrentPosition()) / 4.0;
    }

    public void resetEncoders() {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setDirection(Directions directions) {
        switch (directions) {
            case BACKWARD:
                backRight.setDirection(DcMotor.Direction.REVERSE);
                frontRight.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                break;
            case FORWARD:
                backRight.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                break;
            case RIGHT:
                backRight.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                break;
            case LEFT:
                backRight.setDirection(DcMotor.Direction.REVERSE);
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                break;
            case ROTATE_RIGHT:
                backRight.setDirection(DcMotor.Direction.REVERSE);
                frontRight.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                frontLeft.setDirection(DcMotor.Direction.REVERSE);
                break;
            case ROTATE_LEFT:
                backRight.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                break;
        }

    }

    private void setPowerToMotors(double power)
    {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    private void setPowerToMotors(double power1, double power2, double power3, double power4)
    {
        frontRight.setPower(power1);
        frontLeft.setPower(power2);
        backRight.setPower(power3);
        backLeft.setPower(power4);
    }

    private void setMotorModes()
    {
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveToWithGyro(Directions direction, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower, AutoFunctionsV2 af2) {
        double targetAngle = getAngle();

        resetEncoders();
        setDirection(direction);
        setMotorModes();

        double gain = -0.015, myPow, deviation;
        if (direction == Directions.BACKWARD) {
            gain = 0.015;
        }

        af2.prepareForQueries(maxPower);

        while (getEncoderAverage() < encoderTarget) {

            af2.executeQueries(maxPower);

            deviation = targetAngle - getAngle();
            if (deviation > 180) deviation -= 360;
            else if (deviation < -180) deviation += 360;

            myPow = getCurrentPower((int)getEncoderAverage(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);

            setPowerToMotors(myPow - gain * deviation, myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation);

        }

        setPowerToMotors(0);
    }

    public void turnWithGyro(Directions direction, double angle, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower, AutoFunctionsV2 af2) {
        double targetAngle = angle;

        resetEncoders();
        setDirection(direction);
        setMotorModes();

        double gain = -0.007, myPow, deviation;
        if (direction == Directions.BACKWARD) {
            gain = 0.007;
        }

        af2.prepareForQueries(maxPower);

        while (getEncoderAverage() < encoderTarget) {

            af2.executeQueries(maxPower);

            deviation = targetAngle - getAngle();
            if (deviation > 180) deviation -= 360;
            else if (deviation < -180) deviation += 360;

            myPow = getCurrentPower((int)getEncoderAverage(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);

            setPowerToMotors(myPow - gain * deviation, myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation);

        }

        setPowerToMotors(0);
    }

    public void strafeWithGyro(double patrate, Directions direction, double pow) {
        double targetAngle = 0;
        resetEncoders();
        setDirection(direction);
        double  val = patrate * valpatrat;
        int intval = (int) val;
        frontRight.setTargetPosition(intval);
        frontLeft.setTargetPosition(intval);
        backRight.setTargetPosition(intval);
        backLeft.setTargetPosition(intval);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double gain = .01, accelerateGain = 0.05, myPow = minPower, brakeGain = 0.05;
        setPowerToMotors(myPow);
        while (frontRight.isBusy()) {
            if (myPow < pow) {
                myPow += accelerateGain;
            }
            if (myPow > minPower && frontRight.getTargetPosition() - frontRight.getCurrentPosition() < 500) {
                myPow -= brakeGain;
            }
            double deviation = targetAngle - getAngle();
            lom.telemetry.addData("Deviation", deviation);
            lom.telemetry.update();
            setPowerToMotors(myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation, myPow + gain * deviation);
        }
        setPowerToMotors(0);
    }
}
