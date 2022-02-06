package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoFunc {

    private DcMotor intakeR;
    private DcMotor intakeL;
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor outputmotor;
    private ColorRangeSensor color;
    private CRServo duckServo;
    private AutoMovement am;
    private BNO055IMU imu;
    private LinearOpMode opm;

    int saizecicm = 1000;


    public AutoFunc(DcMotor intakeR, DcMotor intakeL, ColorRangeSensor color, CRServo duckServo, LinearOpMode opm, DcMotor frontRight, DcMotor frontLeft, DcMotor backRight, DcMotor backLeft, AutoMovement am, DcMotor outputmotor)
    {
        this.intakeR = intakeR;
        this.intakeL = intakeL;
        this.color = color;
        this.duckServo = duckServo;
        this.opm = opm;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.outputmotor = outputmotor;
        this.am = am;
    }


//    public void intakein(double power)
//    {
//        intakeR.setPower(-1);
//        intakeL.setPower(1);
//        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm*0.7), 300, 600, power*0.2);
//        if(color.getDistance(DistanceUnit.CM) < 10.0)
//        {
//            intakeR.setPower(1);
//            intakeL.setPower(-1);
//            opm.sleep(200);
//        }
//        intakeR.setPower(0);
//        intakeL.setPower(0);
//    }

    public void duck()
    {
        double accGain = 0.02, p = 0.1;
        while (p <= 0.98) {
            p += accGain;
            duckServo.setPower(p);
        }
        opm.sleep(2500);
        duckServo.setPower(0);
    }

    public void output()
    {
        outputmotor.setPower(1);
        opm.sleep(500);
        outputmotor.setPower(0);
    }

}
