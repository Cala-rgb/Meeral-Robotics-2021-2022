package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeAndOutput {

    DcMotor intakemotor1;
    DcMotor intakemotor2;
    DcMotor outputmotor;

    private CRServo liftServoR = null;
    private CRServo liftServoL = null;
    private CRServo duckServo1 = null, duckServo2 = null;
    private Servo totemServo = null;
    private Servo pivRul = null;
    private CRServo cleste  = null;
    private CRServo desrul = null;
    private Servo rBrat = null;
    private CRServo lift = null;

    private LED ledverde;

    ColorRangeSensor color;

    int posinitial=0;
    double duckSpeed1= -0.2, duckSpeed2 = -0.2;
    boolean luat = false, spinWhenLuat = true;
    double lastTimeLuat=-2000.0, lastTimeChanged = -2000.0,ruletapos=0.85,bratpos=0.1;
    OpMode opm;

    public IntakeAndOutput(DcMotor intakemotor1, DcMotor intakemotor2, DcMotor outputmotor, CRServo lift, CRServo duckServo1, CRServo duckServo2, ColorRangeSensor color, OpMode opm, LED ledverde, CRServo cleste,  Servo rBrat)
    {
        this.intakemotor1=intakemotor1;
        this.intakemotor2=intakemotor2;
        this.outputmotor = outputmotor;
        this.lift = lift;
        this.duckServo1 = duckServo1;
        this.duckServo2 = duckServo2;
        this.totemServo = totemServo;
        this.color = color;
        this.opm = opm;
        this.ledverde = ledverde;
        this.pivRul = pivRul;
        this.cleste = cleste;
        this.desrul = desrul;
        this.rBrat = rBrat;
        posinitial = outputmotor.getCurrentPosition();
    }

    public void verifyAll(double rt, double lt, boolean up, boolean down, boolean lift, boolean start, boolean options, double joy2, boolean start2, double rsy, double rsx, boolean rb, boolean lb, double time)
    {
        turnOnIntake(rt, lt, time);
        //lift(right, left);
        luatobiect(time, opm);
        ratusca(start, options);
        treatOutput(joy2);
        changeSpinWhenLuat(start2, time);
        //ruleta(rsy, rsx);
        turnOnCleste(rb, lb);
        rotireCleste(up, down);
    }

    public void verifyAll(double rt, double lt, boolean up, boolean down, boolean right, boolean left, double joy2, double time)
    {
        turnOnIntake(rt, lt, time);
        //lift(right, left);
        luatobiect(time, opm);
        treatOutput(joy2);
    }

    private void changeSpinWhenLuat(boolean start2, double time) {
        if (start2 && time - lastTimeChanged >= 500.0) {
            lastTimeChanged = time;
            spinWhenLuat = !spinWhenLuat;
        }
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
            outputmotor.setPower(joy2 * -0.6 +0.05);
    }

    void ratusca(boolean start, boolean options) {
        if (start || (luat && spinWhenLuat))
        {
            if(duckSpeed1 < 1){
                duckSpeed1 += 0.035;
            }
            duckServo1.setPower(duckSpeed1);
        }
        else{
            duckSpeed1 = 0.2;
            duckServo1.setPower(0);
        }
        if (options)
        {
            if(duckSpeed2 > -1){
                duckSpeed2 -= 0.035;
            }
            duckServo2.setPower(duckSpeed2);
        }
        else {
            duckSpeed2 = -0.2;
            duckServo2.setPower(0);
        }
    }

    void ruleta(double rsy, double rsx)
    {
        //if(pivRul.getPosition()>=-0.5 && pivRul.getPosition() <=0.5)
        //{
            ruletapos += (rsy*0.003);
            pivRul.setPosition(ruletapos);
        //}
        desrul.setPower(rsx);
    }

    void turnOnCleste(boolean rb, boolean lb)
    {
        if(rb) {
            cleste.setPower(1);
        } else if(lb){
            cleste.setPower(-1);
        } else {
            cleste.setPower(0);
        }
    }

    void rotireCleste(boolean up, boolean down)
    {
        //if(rBrat.getPosition()>-0.95 && rBrat.getPosition()<0.95)
        //{
            if(down) {
                bratpos+=0.005;
            } else if(up) {
                bratpos -= 0.005;
            }
            if(rBrat.getPosition()>0.835)
                bratpos=0.8381;
            rBrat.setPosition(bratpos);
       // }
    }

}
