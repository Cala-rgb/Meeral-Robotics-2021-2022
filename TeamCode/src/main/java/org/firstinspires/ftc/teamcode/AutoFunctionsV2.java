package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoFunctionsV2 {

    private DcMotor intakeR, intakeL, outputmotor;
    private ColorRangeSensor color;
    private CRServo duckServo, liftServoR, liftServoL;
    private LinearOpMode linearOpMode;

    private boolean elevatorEnabled = false, outputmotorEnabled = false, intakeEnabled = false;
    private double elevatorStop = 0.0, outputmotorStop = 0.0;
    private boolean elevatorForward = true, outputmotorForward = true;

    private double initialTime = 0.0;

    public AutoFunctionsV2(DcMotor intakeR, DcMotor intakeL, DcMotor outputmotor, ColorRangeSensor color, CRServo duckServo, CRServo liftServoR, CRServo liftServoL, LinearOpMode lom)
    {
        this.intakeR = intakeR;
        this.intakeL = intakeL;
        this.color = color;
        this.duckServo = duckServo;
        this.outputmotor = outputmotor;
        this.liftServoR = liftServoR;
        this.liftServoL = liftServoL;
        this.linearOpMode = lom;
    }

    public void prepareForQueries(double power) {
        initialTime = linearOpMode.getRuntime();
        if (elevatorEnabled) {
            if (elevatorForward) {
                liftServoR.setPower(-1);
                liftServoL.setPower(1);
            }
            else {
                liftServoR.setPower(1);
                liftServoL.setPower(-1);
            }
        }
        if (outputmotorEnabled) {
            if (outputmotorForward) {
                outputmotor.setPower(-1);
            }
            else {
                outputmotor.setPower(1);
            }
        }
        if (intakeEnabled) {
            if(color.getDistance(DistanceUnit.CM) < 10.0)
            {
                intakeR.setPower(0.7);
                intakeL.setPower(-0.7);
            }
            else {
                intakeR.setPower(-0.8);
                intakeL.setPower(0.8);
            }
        }
    }

    public void executeQueries(double power) {
        if (elevatorEnabled && linearOpMode.getRuntime() - initialTime >= elevatorStop) {
            liftServoR.setPower(0);
            liftServoL.setPower(0);
        }
        if (outputmotorEnabled && linearOpMode.getRuntime() - initialTime >= outputmotorStop) {
            outputmotor.setPower(0);
        }
        if (intakeEnabled) {
            if(color.getDistance(DistanceUnit.CM) < 10.0)
            {
                intakeR.setPower(0.7);
                intakeL.setPower(-0.7);
            }
        }
    }

    public void setElevator(boolean enabled, double time, boolean forward) {
        elevatorEnabled = enabled;
        elevatorStop = time;
        elevatorForward = forward;
    }

    public void setOutputmotor(boolean enabled, double time, boolean forward) {
        outputmotorEnabled = enabled;
        outputmotorStop = time;
        outputmotorForward = forward;
    }

    public void setIntake(boolean enabled) {
        intakeEnabled = enabled;
    }
}
