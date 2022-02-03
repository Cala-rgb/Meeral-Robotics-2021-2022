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

    int posinitial=0, targetPos = 0;
    int merge = 0;
    int val = 1200;
    double duckSpeed= -0.2;
    boolean apsatA = false,luat = false, k = false, apasatB = false;
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

    public void verifyAll(boolean x, boolean b, boolean up, boolean down, boolean right, boolean left, boolean start, boolean a2, boolean b2, double time)
    {
        turnOnIntake(x, b, time);
        lift(right, left);
        totem(up, down);
        luatobiect(time, opm);
        ratusca(start);
        //goToPos(time);
        treatOutput(a2, b2);
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
            totemServo.setPosition(0.5);
        }
        else if(up)
        {
            totemServo.setPosition(0.25);
        }
        else
        {
            totemServo.setPosition(0);
        }
    }

    void turnOnIntake(boolean x,boolean b, double time)
    {
        if(b && !luat){
            intakemotor1.setPower(-0.9);
            intakemotor2.setPower(0.9);
        }
        else if(x || time - lastTimeLuat <= 1000) {
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
        if(color.getDistance(DistanceUnit.CM) < 10.0)
        {
            if (luat == false) {
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

    void treatOutput(boolean a2, boolean b2) {
        if(a2) {
            outputmotor.setPower(0.7);
        }
        else if(b2){
            outputmotor.setPower(-0.7);
        }
        else
            outputmotor.setPower(0);
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

//    void goToPos( double time) {
//        if (k == true && outputmotor.getCurrentPosition() != outputmotor.getTargetPosition()) {
//            apasatB = false;
//            int pos = outputmotor.getTargetPosition();
//            outputmotor.setTargetPosition(pos);
//            outputmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            outputmotor.setPower(-1);
//        }
//        if (k == true && outputmotor.getCurrentPosition() == outputmotor.getTargetPosition()) {
//            outputmotor.setPower(0);
//            luat = false;
//            apasatB = true;
//            k = false;
//
//        }
//        if (luat && time - lastTimeY > 500.0) {
//            lastTimeY = time;
//            if (!k && !apasatB) {
//                k = true;
//                apasatB = true;
//                outputmotor.setTargetPosition(val);
//                outputmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                outputmotor.setPower(-1);
//            } else if (apasatB) {
//                outputmotor.setTargetPosition(0);
//                outputmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                outputmotor.setPower(-1);
//                k = true;
//                apasatB = false;
//            }
//        }
//    }
}
