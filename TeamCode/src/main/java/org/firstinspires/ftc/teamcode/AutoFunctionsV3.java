package org.firstinspires.ftc.teamcode;

import android.graphics.BlurMaskFilter;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoFunctionsV3 {

    private DcMotor intakeR, intakeL, outputmotor;
    private RevColorSensorV3 color, under, under2;
    private CRServo duckServo, liftServoR, liftServoL;
    private LinearOpMode linearOpMode;

    private boolean elevatorEnabled = false, outputmotorEnabled = false, intakeEnabled = false;
    private double elevatorStop = 0.0, outputmotorStop = 0.0, intakeStop = 0.0;
    private boolean elevatorForward = true, outputmotorForward = true, intakeForward = true;

    private double initialTime = 0.0;

    public enum Tasks {NONE, TAKE_FREIGHT, LEAVE_STORAGE};

    private Tasks currentTask = Tasks.NONE;

    double colorsumtotal = 1250.0, alphatotal =850.0;

    //Tasks - regleaza miscarea
    //Queries -  se executa in timpul miscarii

    public AutoFunctionsV3(LinearOpMode lom, DcMotor intakeR, DcMotor intakeL, DcMotor outputmotor, RevColorSensorV3 color, RevColorSensorV3 under, RevColorSensorV3 under2,CRServo duckServo, CRServo liftServoR, CRServo liftServoL)
    {
        this.intakeR = intakeR;
        this.intakeL = intakeL;
        this.color = color;
        this.under = under;
        this.under2 = under2;
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
        else
        {
            liftServoR.setPower(0);
            liftServoL.setPower(0);
        }
        if (outputmotorEnabled) {
            if (outputmotorForward) {
                outputmotor.setPower(-1);
            }
            else {
                outputmotor.setPower(1);
            }
        }
        else
        {
            outputmotor.setPower(0);
        }
        if (intakeEnabled) {
            if(color.getDistance(DistanceUnit.CM) < 10.0)
            {
                intakeR.setPower(0.7);
                intakeL.setPower(-0.7);
            }
            else {
                intakeR.setPower(-1);
                intakeL.setPower(1);
            }
        }
        else {
            intakeR.setPower(0);
            intakeL.setPower(0);
        }
    }

    public void executeQueries(double power) {
        if (elevatorEnabled && linearOpMode.getRuntime() - initialTime >= elevatorStop) {
            liftServoR.setPower(0);
            liftServoL.setPower(0);
        }
        if (outputmotorEnabled && linearOpMode.getRuntime() - initialTime >= outputmotorStop) {
            outputmotor.setPower(-0.15);
        }
        if (intakeEnabled) {
            if(color.getDistance(DistanceUnit.CM) < 10.0) {
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

    public Tasks getCurrentTask() {
        return currentTask;
    }

    public void setTask(Tasks task) {
        currentTask = task;
    }

    public void setColorAndAlpha(double totalcolor, double totalalpha)
    {
        colorsumtotal = totalcolor;
        alphatotal = totalalpha;
    }

    public void updateTask() {
        double red = under.red();
        double blue = under.blue();
        double green = under.green();
        int alpha = under.alpha();
        double red2 = under2.red();
        double blue2 = under2.blue();
        double green2 = under2.green();
        int alpha2 = under2.alpha();
        linearOpMode.telemetry.addData("Red", red);
        linearOpMode.telemetry.addData("Green", green);
        linearOpMode.telemetry.addData("Blue", blue);
        linearOpMode.telemetry.addData("alpha", alpha);
        linearOpMode.telemetry.update();
        if (currentTask == Tasks.LEAVE_STORAGE && ((alpha > alphatotal || red + green + blue > colorsumtotal) || (alpha2 > alphatotal || red2 + green2 + blue2 > colorsumtotal))) {
            currentTask = Tasks.NONE;
        }
        else if (currentTask == Tasks.TAKE_FREIGHT) {
            if(color.getDistance(DistanceUnit.CM) < 10.0) {
                currentTask = Tasks.NONE;
            }
        }
    }
}