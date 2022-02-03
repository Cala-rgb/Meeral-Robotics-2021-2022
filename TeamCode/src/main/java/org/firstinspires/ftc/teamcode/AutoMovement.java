package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutoMovement {
    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private BNO055IMU imu;
    private LinearOpMode lom;
    int valpatrat=1000;
    double minPower = 0.2;
    //trebuie gasita valoarea unui patrat
    public enum Directions {FORWARD, BACKWARD, LEFT, RIGHT, ROTATE_RIGHT, ROTATE_LEFT};
    public AutoMovement (LinearOpMode lom, BNO055IMU imu, DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft) {
        this.lom = lom;
        this.imu = imu;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
    }

    private float getAngle() {
         Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         return angle.firstAngle;
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
    public void driveTo(double patrate, Directions direction, double pow) {
        resetEncoders();
        setDirection(direction);
        double  val=patrate * valpatrat;
        int intval=(int) val;
        frontRight.setTargetPosition(intval);
        frontLeft.setTargetPosition(intval);
        backRight.setTargetPosition(intval);
        backLeft.setTargetPosition(intval);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(pow);
        frontLeft.setPower(pow);
        backRight.setPower(pow);
        backLeft.setPower(pow);
    }

    public void driveToWithGyro(double patrate, Directions direction, double pow) {
        double targetAngle = getAngle();
        resetEncoders();
        setDirection(direction);
        double  val=patrate * valpatrat;
        int intval=(int) val;
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
        double gain = .01, accelerateGain = 0.02, myPow = minPower, brakeGain = 0.05;
        frontRight.setPower(myPow);
        frontLeft.setPower(myPow);
        backRight.setPower(myPow);
        backLeft.setPower(myPow);
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
            frontRight.setPower(myPow + gain * deviation);
            frontLeft.setPower(myPow - gain * deviation);
            backRight.setPower(myPow + gain * deviation);
            backLeft.setPower(myPow - gain * deviation);
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
    public void driveToWithGyro(double patrate, Directions direction, double pow, double time, ServoController sc, boolean ok) {
        double targetAngle = getAngle();
        double startTime = lom.getRuntime();
        resetEncoders();
        setDirection(direction);
        double  val=patrate * valpatrat;
        int intval=(int) val;
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
        double gain = .01, accelerateGain = 0.02, myPow = minPower, brakeGain = 0.05;
        frontRight.setPower(myPow);
        frontLeft.setPower(myPow);
        backRight.setPower(myPow);
        backLeft.setPower(myPow);
        sc.setPowerToServos(-1, ok);
        while (frontRight.isBusy()) {
            if (lom.getRuntime() - startTime >= time) {
                sc.setPowerToServos(0, ok);
            }
            if (myPow < pow) {
                myPow += accelerateGain;
            }
            if (myPow > minPower && frontRight.getTargetPosition() - frontRight.getCurrentPosition() < 500) {
                myPow -= brakeGain;
            }
            double deviation = targetAngle - getAngle();
            lom.telemetry.addData("Deviation", deviation);
            lom.telemetry.addData("Time", lom.getRuntime());
            lom.telemetry.addData("Starttime", startTime);
            lom.telemetry.update();
            frontRight.setPower(myPow + gain * deviation);
            frontLeft.setPower(myPow - gain * deviation);
            backRight.setPower(myPow + gain * deviation);
            backLeft.setPower(myPow - gain * deviation);
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void strafeWithGyro(double patrate, Directions direction, double pow) {
        double targetAngle = getAngle();
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
        frontRight.setPower(myPow);
        frontLeft.setPower(myPow);
        backRight.setPower(myPow);
        backLeft.setPower(myPow);
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
            frontRight.setPower(myPow + gain * deviation);
            frontLeft.setPower(myPow - gain * deviation);
            backRight.setPower(myPow + gain * deviation);
            backLeft.setPower(myPow - gain * deviation);
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    public void rotateTo(float unghi, double pow) {
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double accelerateGain = 0.02, myPow = minPower, brakeGain = 0.15, angleofAction = 50 * pow;
        frontRight.setPower(myPow);
        frontLeft.setPower(myPow);
        backRight.setPower(myPow);
        backLeft.setPower(myPow);
        if (unghi > getAngle()) {
            setDirection(Directions.ROTATE_LEFT);
            while (unghi > getAngle()) {
                if (myPow < pow) {
                    myPow += accelerateGain;
                }
                if (myPow >= minPower && unghi - getAngle() <= angleofAction) {
                    myPow -= brakeGain;
                }
                frontRight.setPower(myPow);
                frontLeft.setPower(myPow);
                backRight.setPower(myPow);
                backLeft.setPower(myPow);
            }
        }
        else {
            setDirection(Directions.ROTATE_RIGHT);
            while (unghi < getAngle()) {
                if (myPow < pow) {
                    myPow += accelerateGain;
                }
                if (myPow > minPower && getAngle() - unghi <= angleofAction) {
                    myPow -= brakeGain;
                }
                frontRight.setPower(myPow);
                frontLeft.setPower(myPow);
                backRight.setPower(myPow);
                backLeft.setPower(myPow);
            }
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

}
