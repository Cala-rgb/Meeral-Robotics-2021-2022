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
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorAdafruitRGB;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Autonom version 3.6", group="Autonom bomba")
public class AutoV3_6 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight, frontLeft, backRight, backLeft, intakeR, intakeL, outputmotor;
    private AutoMovement2 am2;
    private BNO055IMU imu;
    private CRServo duckServo, liftServoR, liftServoL;
    private Servo preloadedServo, totemServo;
    private OpenCvWebcam webcam;
    private ColorRangeSensor color;
    private TeamElementPipeline pipeline;
    private TeamElementPipeline.TeamElementPosition snapshotAnalysis = TeamElementPipeline.TeamElementPosition.LEFT;
    private ServoController sc;
    private AutoFunctionsV2 af2;
    int nouazecigrade = 610;
    int saizecicm = 1000;
    VoltageSensor vs;

    private void getHardware() {

        duckServo = hardwareMap.get(CRServo.class, "duckServo");
        liftServoL = hardwareMap.get(CRServo.class, "lift2");
        liftServoR = hardwareMap.get(CRServo.class, "lift1");
        totemServo = hardwareMap.get(Servo.class, "totemservo");
        preloadedServo = hardwareMap.get(Servo.class, "preloadedservo");

        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft  = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        intakeR = hardwareMap.get(DcMotor.class, "intakeR");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        outputmotor = hardwareMap.get(DcMotor.class, "outputmotor");
        color = hardwareMap.get(ColorRangeSensor.class, "color");

        vs = hardwareMap.voltageSensor.iterator().next();
        duckServo.setDirection(DcMotorSimple.Direction.REVERSE);
        //?
    }

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new TeamElementPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

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
        while ((!isStarted() && !isStopRequested()) || !imu.isGyroCalibrated())
        {
            snapshotAnalysis = pipeline.getAnalysis();
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();
            sleep(50);
            idle();
        }

        if(snapshotAnalysis == TeamElementPipeline.TeamElementPosition.LEFT) {
            af2.setElevator(true, 0, true);
        }
        else if(snapshotAnalysis == TeamElementPipeline.TeamElementPosition.CENTER){
            af2.setElevator(true, 0.75, true);
        }
        else {
            af2.setElevator(true, 3, true);
        }

        getHardware();
        sc = new ServoController(liftServoR, liftServoL, outputmotor);
        am2 = new AutoMovement2(this, imu, frontRight, frontLeft, backRight, backLeft, vs);
        af2 = new AutoFunctionsV2(intakeR, intakeL, outputmotor, color, duckServo, liftServoR, liftServoL);

        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        runtime.reset();

        double voltage = vs.getVoltage();
        double power = 11.0 / voltage;

        //Setam servourile la pozitilor lor

        preloadedServo.setPosition(0.8);
        totemServo.setPosition(0);

        //Mergem in fata la shipping hub

        am2.driveToWithGyro(AutoMovement2.Directions.BACKWARD, (int) (saizecicm * 1.9), 300, 1600, power * 0.5, af2);

        sleep(200);

        //Lasam cubul preloaded

        preloadedServo.setPosition(0);
        sleep(500);
        preloadedServo.setPosition(0.8);

        sleep(100);

        //Dam inapoi

        af2.setElevator(true, 2, false);

        am2.driveToWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm * 0.95), 300, 650, power, af2);

        af2.setElevator(false, 0, true);

        am2.turnWithGyro(AutoMovement2.Directions.FORWARD, 0, (int) (saizecicm * 2.2), 300, 1900, power, af2);
        sleep(100);

        //Luam cub

        af2.setIntake(true);
        am2.driveToWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm*0.3), 100, 200, power * 0.35, af2);
        sleep(500);
        am2.driveToWithGyro(AutoMovement2.Directions.BACKWARD, (int) (saizecicm*0.25), 100, 150, power, af2);
        sleep(100);
        af2.setIntake(false);

        //Punem cubul sus

        af2.setOutputmotor(true, 0.75, true);
        am2.driveToWithGyro(AutoMovement2.Directions.BACKWARD, (int) (saizecicm*1), 300, 700, power, af2);
        am2.turnWithGyro(AutoMovement2.Directions.BACKWARD, -65, (int) (saizecicm*1.92), 300, 1600, power, af2);

        outputmotor.setPower(-1);
        sleep(1000);
        outputmotor.setPower(0);

        af2.setOutputmotor(true, 1, false);

        am2.driveToWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm * 0.95), 300, 650, power, af2);
        am2.turnWithGyro(AutoMovement2.Directions.FORWARD, 0, (int) (saizecicm * 2.3), 300, 2000, power, af2);

        af2.setOutputmotor(false, 0, false);
        //cub1
        /*am.driveAndIntake(AutoMovement.Directions.FORWARD, (int) (saizecicm*0.3), 100, 200, power*0.35, intakeL, intakeR, color, power);
        sleep(500);
        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm*0.25), 100, 150, power);
        sleep(100);
        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm*1), 300, 700, power, 0.75, sc, false, true);
        am.turnB(-65, AutoMovement.Directions.BACKWARD, (int) (saizecicm*1.92), 300, 1600, power, 0.75, sc, false, true);
        outputmotor.setPower(-1);
        sleep(1000);
        outputmotor.setPower(0);
        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm * 0.95), 300, 650, power, 1, sc,  false, false);
        am.turn(0, AutoMovement.Directions.FORWARD, (int) (saizecicm * 2.4), 300, 2100, power, 1.5, sc, false, false);
        //cub2
        am.driveAndIntake(AutoMovement.Directions.FORWARD, (int) (saizecicm*0.3), 100, 200, power*0.35, intakeL, intakeR, color, power);
        sleep(500);
        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm*0.25), 100, 150, power);
        sleep(100);
        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm*1), 300, 700, power, 0.75, sc, false, true);
        am.turnB(-65, AutoMovement.Directions.BACKWARD, (int) (saizecicm*1.92), 300, 1600, power, 0.75, sc, false, true);
        outputmotor.setPower(-1);
        sleep(1000);
        outputmotor.setPower(0);
        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm * 0.95), 300, 650, power, 1, sc,  false, false);
        am.turn(0, AutoMovement.Directions.FORWARD, (int) (saizecicm * 2.5), 300, 2200, power, 1.5, sc, false, false);
        //cub3
        am.driveAndIntake(AutoMovement.Directions.FORWARD, (int) (saizecicm*0.3), 100, 200, power*0.35, intakeL, intakeR, color, power);
        sleep(500);
        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm*0.25), 100, 150, power);
        sleep(100);
        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm*1), 300, 700, power, 0.75, sc, false, true);
        am.turnB(-65, AutoMovement.Directions.BACKWARD, (int) (saizecicm*1.92), 300, 1600, power, 0.75, sc, false, true);
        outputmotor.setPower(-1);
        sleep(1000);
        outputmotor.setPower(0);
        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm * 0.95), 300, 650, power, 1, sc,  false, false);
        am.turn(0, AutoMovement.Directions.FORWARD, (int) (saizecicm * 2.5), 300, 2200, power, 1.5, sc, false, false);*/
    }
}
