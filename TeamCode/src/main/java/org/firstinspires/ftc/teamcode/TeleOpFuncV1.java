package org.firstinspires.ftc.teamcode;

import android.graphics.BlurMaskFilter;
import android.graphics.Color;
import android.graphics.Path;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TeleOpFuncV1 {

    private DcMotor intakeR, intakeL, outputmotor;
    private RevColorSensorV3 color, under;
    private CRServo duckServo, lift;
    private OpMode OpMode;

    private boolean elevatorEnabled = false, outputmotorEnabled = false, intakeEnabled = false;
    private double elevatorStop = 0.0, outputmotorStop = 0.0, intakeStop = 0.0;
    private boolean elevatorForward = true, outputmotorForward = true, intakeForward = true;

    private double initialTime = 0.0;

    public enum Tasks {NONE, TAKE_FREIGHT, LEAVE_STORAGE};

    private Tasks currentTask = Tasks.NONE;

    //Tasks - regleaza miscarea
    //Queries -  se executa in timpul miscarii

    public TeleOpFuncV1(OpMode lom, DcMotor intakeR, DcMotor intakeL, DcMotor outputmotor, RevColorSensorV3 color, RevColorSensorV3 under, CRServo duckServo, CRServo lift)
    {
        this.intakeR = intakeR;
        this.intakeL = intakeL;
        this.color = color;
        this.under = under;
        this.duckServo = duckServo;
        this.outputmotor = outputmotor;
        this.lift = lift;
        this.OpMode = lom;
    }

    public void prepareForQueries(double power) {
        initialTime = OpMode.getRuntime();
        if (elevatorEnabled) {
            if (elevatorForward) {
                lift.setPower(-1);
            }
            else {
                lift.setPower(1);
            }
        }
        else
        {
            lift.setPower(0);
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
            if(color.getDistance(DistanceUnit.CM) < 5.0)
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
        if (elevatorEnabled && OpMode.getRuntime() - initialTime >= elevatorStop) {
                lift.setPower(0);
        }
        if (outputmotorEnabled && OpMode.getRuntime() - initialTime >= outputmotorStop) {
            outputmotor.setPower(0.1);
        }
        if (intakeEnabled) {
            if(color.getDistance(DistanceUnit.CM) < 5.0)
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

    public Tasks getCurrentTask() {
        return currentTask;
    }

    public void setTask(Tasks task) {
        currentTask = task;
    }

    public void updateTask() {
        double red = under.red();
        double blue = under.blue();
        double green = under.green();
        int alpha = under.alpha();
        /*OpMode.telemetry.addData("Red", red);
        OpMode.telemetry.addData("Green", green);
        OpMode.telemetry.addData("Blue", blue);
        OpMode.telemetry.addData("alpha", alpha);
        OpMode.telemetry.update();*/
        if (currentTask == Tasks.LEAVE_STORAGE && (alpha > 800 || red + green + blue > 1450.0)) {
            currentTask = Tasks.NONE;
        }
        else if (currentTask == Tasks.TAKE_FREIGHT) {
            if(color.getDistance(DistanceUnit.CM) < 5.0) {
                currentTask = Tasks.NONE;
            }
        }
    }
}