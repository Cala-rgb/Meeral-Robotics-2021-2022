package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Movement {

    private DcMotor fl,fr,bl,br,outputmotor;

    double lastTimeB = 0.0;

    boolean inMove = false;

    private BNO055IMU imu;

    double minPower;
    VoltageSensor vs;
    ColorSensor under;

    OpMode opm;

    public Movement(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotor outputmotor, BNO055IMU imu, VoltageSensor vs, ColorSensor under, OpMode opm)
    {
        this.fl=fl;
        this.fr=fr;
        this.bl=bl;
        this.br=br;
        this.outputmotor = outputmotor;
        this.imu = imu;
        this.minPower = 0.3 * (11 / vs.getVoltage());
        this.vs= vs;
        this.under = under;
        this.opm = opm;
    }

    void break_func()
    {
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
        fl.setPower(0);
    }

    void move(double forwardpower, double backwardpower, double steer, double strafe, double rsthr, double pow)
    {
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        double throttle = -forwardpower+backwardpower+rsthr;
        fl.setPower((throttle-steer*0.8-strafe)*pow);
        fr.setPower((throttle+steer*0.8+strafe)*pow);
        bl.setPower((throttle-steer*0.8+strafe)*pow);
        br.setPower((throttle+steer*0.8-strafe)*pow);
    }

    void bumbersteering(double pow)
    {
        fl.setPower(-pow);
        fr.setPower(pow);
        bl.setPower(-pow);
        br.setPower(pow);
    }

    private void setPowerToMotors(double power1, double power2, double power3, double power4)
    {
        fr.setPower(power1);
        fl.setPower(power2);
        br.setPower(power3);
        bl.setPower(power4);
    }

    private void setPowerToMotors(double power)
    {
        fr.setPower(power);
        fl.setPower(power);
        br.setPower(power);
        bl.setPower(power);
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
        return (fr.getCurrentPosition() + fl.getCurrentPosition() + br.getCurrentPosition() + bl.getCurrentPosition()) / 4.0;
    }

    public void resetEncoders() {
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setDirection(boolean forward)
    {
        if(forward) {
            br.setDirection(DcMotor.Direction.FORWARD);
            fr.setDirection(DcMotor.Direction.FORWARD);
            bl.setDirection(DcMotor.Direction.REVERSE);
            fl.setDirection(DcMotor.Direction.REVERSE);
        } else {
            br.setDirection(DcMotor.Direction.REVERSE);
            fr.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.FORWARD);
            fl.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    private void setMotorModes()
    {
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    boolean trecutDeLinie()
    {
        if((under.red() + under.green() + under.blue()) > 1250.0 && under.alpha() > 600)
            return true;
        return false;
    }

    void autoMove(boolean b)
    {
        if(b && opm.getRuntime()-lastTimeB>0.5)
        {
            lastTimeB = opm.getRuntime();
            inMove = true;
            double targetAngle = getAngle();

            boolean forward = false;

            resetEncoders();
            setDirection(forward);
            setMotorModes();

            double gain = -0.015, myPow, deviation;
            if (!forward) {
                gain = 0.015;
            }

            double initTime = opm.getRuntime();

            if(opm.gamepad1.b && opm.getRuntime()-lastTimeB>0.5)
                return ;



            while (getEncoderAverage() < 2200 && inMove) {

                if(getEncoderAverage()>=300)
                {
                    targetAngle = -66;

                }



                if(opm.getRuntime()-initTime < 0.2)
                {
                    outputmotor.setPower(1);
                }

                deviation = targetAngle - getAngle();
                if (deviation > 180) deviation -= 360;
                else if (deviation < -180) deviation += 360;

                double maxPow =(11/vs.getVoltage());

                myPow = getCurrentPower((int)getEncoderAverage(), 300, 1900, 2200, maxPow);

                setPowerToMotors(myPow - gain * deviation, myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation);
                opm.telemetry.addData("ok",getEncoderAverage());
                opm.telemetry.update();

                if(getEncoderAverage()>=2200)
                    inMove = false;

                if(opm.gamepad1.b && opm.getRuntime()-lastTimeB>0.5)
                    return ;
            }

            opm.telemetry.addData("ok",getEncoderAverage());
            opm.telemetry.update();

            setPowerToMotors(0);

            if(opm.gamepad1.b && opm.getRuntime()-lastTimeB>0.5)
                return ;

            outputmotor.setPower(-1);
            double time = opm.getRuntime();
            while(opm.getRuntime()-time<1)
                outputmotor.setPower(1);
            outputmotor.setPower(0);

            if(opm.gamepad1.b && opm.getRuntime()-lastTimeB>0.5)
                return ;

            inMove = true;

            targetAngle = getAngle();

            forward = true;

            resetEncoders();
            setDirection(forward);
            setMotorModes();

            gain = -0.015;
            if (!forward) {
                gain = 0.015;
            }

            initTime = opm.getRuntime();

            if(opm.gamepad1.b && opm.getRuntime()-lastTimeB>0.5)
                return ;

            while (getEncoderAverage() < 3000 && inMove) {

                if(getEncoderAverage() >= 605)
                    targetAngle = 0;

                if(opm.getRuntime()-initTime < 2) {
                    outputmotor.setPower(-1);
                }
                deviation = targetAngle - getAngle();
                if (deviation > 180) deviation -= 360;
                else if (deviation < -180) deviation += 360;

                myPow = getCurrentPower((int)getEncoderAverage(), 300, 2700, 3000, (11/vs.getVoltage()));

                setPowerToMotors(myPow - gain * deviation, myPow + gain * deviation, myPow - gain * deviation, myPow + gain * deviation);

                if(getEncoderAverage()>=3000)
                    inMove = false;

                if(opm.gamepad1.b && opm.getRuntime()-lastTimeB>0.5)
                    return ;
            }
        }
    }

}
