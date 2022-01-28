/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="Autonom version 2", group="Auto demo")
public class AutoV2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight, frontLeft, backRight, backLeft, intakeR, intakeL;
    private AutoMovement am;
    private BNO055IMU imu;
    private CRServo duckServo;
    VoltageSensor vs;

    private void getHardware() {

        duckServo = hardwareMap.get(CRServo.class, "duckServo");

        frontRight = hardwareMap.get(DcMotor.class, "fr");
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");

        intakeR = hardwareMap.get(DcMotor.class,"intakeR");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);

        vs = hardwareMap.voltageSensor.iterator().next();
    }

    private void telem()
    {
        while(frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy())
        {
            telemetry.addData("frontRight:", frontRight.getCurrentPosition());
            telemetry.addData("frontLeft:", frontLeft.getCurrentPosition());
            telemetry.addData("backRight:", backRight.getCurrentPosition());
            telemetry.addData("backLeft:", backLeft.getCurrentPosition());
            telemetry.addData("frontRightTarget:", frontRight.getTargetPosition());
            telemetry.addData("frontLeftTarget:", frontLeft.getTargetPosition());
            telemetry.addData("backRightTarget:", backRight.getTargetPosition());
            telemetry.addData("backLeftTarget:", backLeft.getTargetPosition());
            telemetry.addData("ceva", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            telemetry.update();
        }
    }


    void iacub(double pow)
    {

        am.driveTo(3.2, AutoMovement.Directions.FORWARD, pow);
        telem();
        //am.resetEncoders();
        intakeR.setPower(0.5);
        intakeL.setPower(0.5);
        sleep(200);
        am.driveTo(0.25, AutoMovement.Directions.FORWARD, 0.2*pow);
        telem();
        //am.resetEncoders();
        intakeR.setPower(0);
        intakeL.setPower(0);
        sleep(500);
        am.driveTo(0.05, AutoMovement.Directions.ROTATE_LEFT, 0.2*pow);
        telem();
        am.driveTo(4.5, AutoMovement.Directions.BACKWARD, 0.5*pow);
        telem();
        //am.resetEncoders();
        am.driveTo(0.75, AutoMovement.Directions.ROTATE_RIGHT, 0.8*pow);
        telem();
        //am.resetEncoders();
        am.driveTo(0.6, AutoMovement.Directions.FORWARD, 0.8*pow);
        telem();
        //am.resetEncoders();
        intakeR.setPower(-0.5);
        intakeL.setPower(-0.5);
        sleep(1000);
        intakeR.setPower(0);
        intakeL.setPower(0);
        sleep(200);
        am.driveTo(0.75, AutoMovement.Directions.BACKWARD, 0.8*pow);
        telem();
        //am.resetEncoders();
        am.driveTo(0.85, AutoMovement.Directions.ROTATE_LEFT, 0.8*pow);
        telem();
        //am.resetEncoders();
        am.driveTo(1.3, AutoMovement.Directions.FORWARD, 0.8*pow);
        telem();
        //am.resetEncoders();
    }
    private void intakeout()
    {
        intakeR.setPower(-1);
        intakeL.setPower(-1);
        sleep(500);
        intakeR.setPower(0);
        intakeL.setPower(0);
    }

    private void intakein(double power)
    {
        intakeR.setPower(1);
        intakeL.setPower(1);
        am.driveToWithGyro(0.1, AutoMovement.Directions.FORWARD, power*0.2);
        intakeR.setPower(0);
        intakeL.setPower(0);
    }

    private void duck()
    {
        duckServo.setPower(1);
        sleep(5000);
        duckServo.setPower(0);
    }

    @Override
    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        getHardware();
        am = new AutoMovement(this, imu, frontRight, frontLeft, backRight, backLeft);

        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double voltage = vs.getVoltage();
        double power= 11.0/voltage;
        am.driveToWithGyro(0.8, AutoMovement.Directions.FORWARD, power);
        duck();
        am.rotateTo(90,power*0.5);
        am.driveToWithGyro(0.6, AutoMovement.Directions.FORWARD, power);
        intakeout();
        am.driveToWithGyro(0.65, AutoMovement.Directions.BACKWARD, power);
        am.rotateTo(179,power*0.5);
        am.driveToWithGyro(4.4, AutoMovement.Directions.FORWARD, power);
        intakein(power);
        am.rotateTo(0,power*0.5);
        am.driveToWithGyro(0.25, AutoMovement.Directions.RIGHT,power * 0.4);
        am.driveToWithGyro(4.5, AutoMovement.Directions.FORWARD,power);
        am.rotateTo(90, power*0.5);
        am.driveToWithGyro(0.7, AutoMovement.Directions.FORWARD, power);
        intakeout();
        am.driveToWithGyro(0.8, AutoMovement.Directions.BACKWARD, power);
        am.rotateTo(179, power*0.5);
        am.driveToWithGyro(4, AutoMovement.Directions.FORWARD, power);
        sleep(2000);

    }
}
