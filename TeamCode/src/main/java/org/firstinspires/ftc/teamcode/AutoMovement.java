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

public class AutoMovement {
    private DcMotor frontRight, frontLeft, backRight, backLeft;
    private BNO055IMU imu;
    private LinearOpMode lom;
    private double lastAngle = 0.0;
    int valpatrat=1000;
    double minPower;
    VoltageSensor vs;

    public enum Directions {FORWARD, BACKWARD, LEFT, RIGHT, ROTATE_RIGHT, ROTATE_LEFT};
    public AutoMovement (LinearOpMode lom, BNO055IMU imu, DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, VoltageSensor vs) {
        this.lom = lom;
        this.imu = imu;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.vs = vs;
        this.minPower = 0.3*(11/vs.getVoltage());
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

    private void setTargetVal()
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

    public void driveTo(Directions direction, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower) {
        double startTime = lom.getRuntime();
        resetEncoders();
        setDirection(direction);
        setTargetVal();
        double myPow;
        while (frontRight.getCurrentPosition() <encoderTarget) {
            myPow = getCurrentPower(frontRight.getCurrentPosition(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);
            lom.telemetry.addData("Time", lom.getRuntime());
            lom.telemetry.addData("Starttime", startTime);
            lom.telemetry.update();
            setPowerToMotors(myPow);
        }
        setPowerToMotors(0);
    }

    public void driveBToWithGyro(Directions direction, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower) {
        double targetAngle = getAngle();
        double startTime = lom.getRuntime();
        resetEncoders();
        setDirection(direction);
        setTargetVal();
        double gain = .015, myPow;
        while (frontRight.getCurrentPosition() <encoderTarget) {
            myPow = getCurrentPower(frontRight.getCurrentPosition(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);
            double deviation = targetAngle - getAngle();
            lom.telemetry.addData("Deviation", deviation);
            lom.telemetry.addData("Time", lom.getRuntime());
            lom.telemetry.addData("Starttime", startTime);
            lom.telemetry.update();
            setPowerToMotors(myPow -gain * deviation, myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation);
        }
        setPowerToMotors(0);
    }

    public void driveBToWithGyro(Directions direction, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower, double time, ServoController sc, boolean ok, boolean inv) {
        double targetAngle = getAngle();
        double startTime = lom.getRuntime();
        resetEncoders();
        setDirection(direction);
        setTargetVal();
        double gain = .015, myPow;
        if(inv == true)
            sc.setPowerToServos(-maxPower, ok);
        else
            sc.setPowerToServos(maxPower, ok);
        while (frontRight.getCurrentPosition() <encoderTarget) {
            if (lom.getRuntime() - startTime >= time) {
                sc.setPowerToServos(0, ok);
            }
            myPow = getCurrentPower(frontRight.getCurrentPosition(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);
            double deviation = targetAngle - getAngle();
            lom.telemetry.addData("Deviation", deviation);
            lom.telemetry.addData("Time", lom.getRuntime());
            lom.telemetry.addData("Starttime", startTime);
            lom.telemetry.update();
            setPowerToMotors(myPow -gain * deviation, myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation);
        }
        setPowerToMotors(0);
    }

    public void turnB(double angle, Directions direction, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower) {
        double targetAngle = angle;
        double startTime = lom.getRuntime();
        resetEncoders();
        setDirection(direction);
        setTargetVal();
        double gain = .015, myPow;
        while (frontRight.getCurrentPosition() <encoderTarget) {
            myPow = getCurrentPower(frontRight.getCurrentPosition(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);
            double deviation = targetAngle - getAngle();
            lom.telemetry.addData("Deviation", deviation);
            lom.telemetry.addData("Time", lom.getRuntime());
            lom.telemetry.addData("Starttime", startTime);
            lom.telemetry.update();
            setPowerToMotors(myPow -gain * deviation, myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation);
        }
        setPowerToMotors(0);
    }

    public void driveToWithGyro(Directions direction, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower) {
        double targetAngle = getAngle();
        double startTime = lom.getRuntime();
        resetEncoders();
        setDirection(direction);
        setTargetVal();
        double gain = .015, myPow;
        while (frontRight.getCurrentPosition() <encoderTarget) {
            myPow = getCurrentPower(frontRight.getCurrentPosition(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);
            double deviation = targetAngle - getAngle();
            if (deviation > 180) deviation -= 360;
            else if (deviation < -180) deviation += 360;
            lom.telemetry.addData("Deviation", deviation);
            lom.telemetry.addData("Time", lom.getRuntime());
            lom.telemetry.addData("Starttime", startTime);
            lom.telemetry.update();
            setPowerToMotors(myPow +gain * deviation, myPow - gain * deviation, myPow + gain * deviation, myPow - gain * deviation);
        }
        setPowerToMotors(0);
    }

    public void driveToWithGyro(Directions direction, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower, double time, ServoController sc, boolean ok, boolean inv) {
        double targetAngle = getAngle();
        double startTime = lom.getRuntime();
        resetEncoders();
        setDirection(direction);
        setTargetVal();
        double gain = .015, myPow;
        if(inv == true)
            sc.setPowerToServos(-maxPower, ok);
        else
            sc.setPowerToServos(maxPower, ok);
        while (frontRight.getCurrentPosition() <encoderTarget) {
            if (lom.getRuntime() - startTime >= time) {
                sc.setPowerToServos(0, ok);
            }
            myPow = getCurrentPower(frontRight.getCurrentPosition(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);
            double deviation = targetAngle - getAngle();
            if (deviation > 180) deviation -= 360;
            else if (deviation < -180) deviation += 360;
            lom.telemetry.addData("Deviation", deviation);
            lom.telemetry.addData("Time", lom.getRuntime());
            lom.telemetry.addData("Starttime", startTime);
            lom.telemetry.update();
            setPowerToMotors(myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation, myPow - gain * deviation);
        }
        setPowerToMotors(0);
    }

    public void turnB(double angle, Directions direction, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower, double time, ServoController sc, boolean ok, boolean inv) {
        double targetAngle = angle;
        double startTime = lom.getRuntime();
        resetEncoders();
        setDirection(direction);
        setTargetVal();
        double gain = .015, myPow;
        if(inv == true)
            sc.setPowerToServos(-maxPower, ok);
        else
            sc.setPowerToServos(maxPower, ok);
        while (frontRight.getCurrentPosition() <encoderTarget) {
            if (lom.getRuntime() - startTime >= time) {
                sc.setPowerToServos(0, ok);
            }
            myPow = getCurrentPower(frontRight.getCurrentPosition(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);
            double deviation = targetAngle - getAngle();
            lom.telemetry.addData("Deviation", deviation);
            lom.telemetry.addData("Time", lom.getRuntime());
            lom.telemetry.addData("Starttime", startTime);
            lom.telemetry.update();
            setPowerToMotors(myPow -gain * deviation, myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation);
        }
        setPowerToMotors(0);
    }
    public void turn(double angle, Directions direction, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower, double time, ServoController sc, boolean ok, boolean inv) {
        double targetAngle = angle;
        double startTime = lom.getRuntime();
        resetEncoders();
        setDirection(direction);
        setTargetVal();
        double gain = .015, myPow;
        if(inv == true)
            sc.setPowerToServos(-maxPower, ok);
        else
            sc.setPowerToServos(maxPower, ok);
        while (frontRight.getCurrentPosition() <encoderTarget) {
            if (lom.getRuntime() - startTime >= time) {
                sc.setPowerToServos(0, ok);
            }
            myPow = getCurrentPower(frontRight.getCurrentPosition(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);
            double deviation = targetAngle - getAngle();
            lom.telemetry.addData("Deviation", deviation);
            lom.telemetry.addData("Time", lom.getRuntime());
            lom.telemetry.addData("Starttime", startTime);
            lom.telemetry.update();
            setPowerToMotors(myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation, myPow - gain * deviation);
        }
        setPowerToMotors(0);
    }

    public void turn(double angle, Directions direction, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower) {
        double targetAngle = angle;
        double startTime = lom.getRuntime();
        resetEncoders();
        setDirection(direction);
        setTargetVal();
        double gain = .015, myPow;
        while (frontRight.getCurrentPosition() <encoderTarget) {
            myPow = getCurrentPower(frontRight.getCurrentPosition(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);
            double deviation = targetAngle - getAngle();
            lom.telemetry.addData("Deviation", deviation);
            lom.telemetry.addData("Time", lom.getRuntime());
            lom.telemetry.addData("Starttime", startTime);
            lom.telemetry.update();
            setPowerToMotors(myPow +gain * deviation, myPow - gain * deviation, myPow + gain * deviation, myPow - gain * deviation);
        }
        setPowerToMotors(0);
    }



    public void driveAndIntake(Directions direction, int encoderTarget, int encoderStopAccelerate, int encoderStartBrake, double maxPower, DcMotor intakeL, DcMotor intakeR, ColorRangeSensor color, double power) {
        double targetAngle = getAngle();
        resetEncoders();
        setDirection(direction);
        setTargetVal();
        double gain = .05, myPow, timeLuat = 0.0;
        intakeR.setPower(-0.85 * power);
        intakeL.setPower(0.85 * power);
        while (frontRight.getCurrentPosition() < encoderTarget) {
            if(color.getDistance(DistanceUnit.CM) < 10.0)
            {
                intakeR.setPower(0.7 * power);
                intakeL.setPower(-0.7 * power);
            }
            myPow = getCurrentPower(frontRight.getCurrentPosition(), encoderStopAccelerate, encoderStartBrake, encoderTarget, maxPower);
            double deviation = targetAngle - getAngle();
            if (deviation > 180) deviation -= 360;
            else if (deviation < -180) deviation += 360;
            lom.telemetry.addData("Deviation", deviation);
            lom.telemetry.update();
            setPowerToMotors(myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation, myPow - gain * deviation);
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

    public void rotateTo(float unghi, double pow) {
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double accelerateGain = 0.02, myPow = minPower, brakeGain = 0.15, angleofAction = 50 * pow;
        setPowerToMotors(myPow);
        if (unghi > getAngle()) {
            setDirection(Directions.ROTATE_LEFT);
            while (unghi > getAngle()) {
                if (myPow < pow) {
                    myPow += accelerateGain;
                }
                if (myPow >= minPower && unghi - getAngle() <= angleofAction) {
                    myPow -= brakeGain;
                }
                setPowerToMotors(myPow);
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
                setPowerToMotors(myPow);
            }
        }
        setPowerToMotors(0);
    }

}
