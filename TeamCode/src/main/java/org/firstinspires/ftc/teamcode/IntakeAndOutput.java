package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeAndOutput {

    DcMotor intakemotor1;
    DcMotor intakemotor2;
    DcMotor outputmotor;

    private CRServo liftServoR = null;
    private CRServo liftServoL = null;
    private CRServo duckServo = null;
    private Servo totemServo = null;

    ColorRangeSensor color;

    int posinitial=0;
    double duckSpeed= -0.2;
    boolean luat = false;
    double lastTimeLuat=-2000.0;
    OpMode opm;

    public IntakeAndOutput(DcMotor intakemotor1, DcMotor intakemotor2, DcMotor outputmotor, CRServo liftServoR, CRServo liftServoL, Servo totemServo, CRServo duckServo, ColorRangeSensor color, OpMode opm)
    {
        this.intakemotor1=intakemotor1;
        this.intakemotor2=intakemotor2;
        this.outputmotor = outputmotor;
        this.liftServoR = liftServoR;
        this.liftServoL = liftServoL;
        this.duckServo = duckServo;
        this.totemServo = totemServo;
        this.color = color;
        this.opm = opm;
        posinitial = outputmotor.getCurrentPosition();
    }

    public void verifyAll(double rt, double lt, boolean up, boolean down, boolean right, boolean left, boolean start, double joy2, double time)
    {
        turnOnIntake(rt, lt, time);
        lift(right, left);
        totem(up, down);
        luatobiect(time, opm);
        ratusca(start);
        treatOutput(joy2);
    }

    public void verifyAll(double rt, double lt, boolean up, boolean down, boolean right, boolean left, double joy2, double time)
    {
        turnOnIntake(rt, lt, time);
        lift(right, left);
        totem(up, down);
        luatobiect(time, opm);
        treatOutput(joy2);
    }

    void lift(boolean right, boolean left)
    {
        if(left)
        {
            liftServoR.setPower(1);
            liftServoL.setPower(-1);
        }
        else if(right)
        {
            liftServoR.setPower(-1);
            liftServoL.setPower(1);
        }
        else
        {
            liftServoR.setPower(0);
            liftServoL.setPower(0);
        }
    }

    void totem(boolean up, boolean down)
    {
        if(down)
        {
            totemServo.setPosition(0.4);
        }
        else if(up)
        {
            totemServo.setPosition(0.2);
        }
        else
        {
            totemServo.setPosition(0);
        }
    }

    void turnOnIntake(double rt, double lt, double time)
    {
        if(rt != 0 && !luat){
            intakemotor1.setPower(-0.7 * rt);
            intakemotor2.setPower(0.7 * rt);
        }
        else if(lt != 0 || time - lastTimeLuat <= 1000) {
            intakemotor1.setPower(0.4);
            intakemotor2.setPower(-0.4);
        }
        else {
            intakemotor1.setPower(0);
            intakemotor2.setPower(0);
        }
    }


    void luatobiect(double time, OpMode opm)
    {
        if(color.getDistance(DistanceUnit.CM) < 5.0)
        {
            if (!luat) {
                lastTimeLuat = time;
                opm.gamepad1.rumble(1.0, 1.0, 300);
                opm.gamepad2.rumble(1.0, 1.0, 300);
            }
            luat = true;
        }
        else {
            luat = false;
        }
    }

    void treatOutput(double joy2) {
            outputmotor.setPower(joy2 * -0.8);
    }

    void ratusca(boolean start) {
        if (start || luat)
        {
            if(duckSpeed > -1){
                duckSpeed -= 0.035;
            }
            duckServo.setPower(duckSpeed);
        }
        else{
            duckSpeed = -0.2;
            duckServo.setPower(0);
        }
    }

}
