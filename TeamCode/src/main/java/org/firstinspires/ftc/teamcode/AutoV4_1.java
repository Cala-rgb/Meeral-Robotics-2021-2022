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

@Autonomous(name = "Autonom version 4_1", group="Autonom bomba")
public class AutoV4_1 extends LinearOpMode {

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
    double uptime;
    double startVoltage,minVoltage=20.0;
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

    private double getPower()
    {
        double voltage = vs.getVoltage();
        double power = 11.5 / voltage;
        return power;
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

        getHardware();
        sc = new ServoController(liftServoR, liftServoL, outputmotor);
        am2 = new AutoMovement2(this, imu, frontRight, frontLeft, backRight, backLeft, vs);
        af2 = new AutoFunctionsV2(intakeR, intakeL, outputmotor, color, duckServo, liftServoR, liftServoL, this);


        if(snapshotAnalysis == TeamElementPipeline.TeamElementPosition.LEFT) {
            uptime = 0;
        }
        else if(snapshotAnalysis == TeamElementPipeline.TeamElementPosition.CENTER){
            uptime = 0.9;
        }
        else {
            uptime = 3;
        }

        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        runtime.reset();

        //Setam servourile la pozitilor lor

        preloadedServo.setPosition(1);
        totemServo.setPosition(0);

        //Mergem in fata la shipping hub

        af2.setElevator(true, uptime, true);

        am2.turnWithGyro(AutoMovement2.Directions.BACKWARD, -70, (int) (saizecicm * 1.4), 300, 1150, getPower() * 0.45, af2);

        sleep(200);

        //Lasam cubul preloaded

        preloadedServo.setPosition(0);
        sleep(500);
        preloadedServo.setPosition(1);
        liftServoR.setPower(0);
        liftServoL.setPower(0);

        //Dam inapoi

        af2.setElevator(true, 2, false);

        //am2.driveToAndTurnWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm*2.6), 300, 2400, getPower(), af2, 0, 650);
        am2.driveToAndTurnAndStrafeWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm*2.5), 300, 3700, getPower(), af2, 0, 600, 1100, 800);


        af2.setElevator(false, 0, true);
        liftServoL.setPower(0);
        liftServoR.setPower(0);

        //Luam cub

        af2.setIntake(true);
        am2.driveToWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm*0.5), 100, 400, getPower() * 0.35, af2);
        sleep(100);
        am2.turnWithGyro(AutoMovement2.Directions.BACKWARD, 5,(int) (saizecicm*0.55), 100, 150, getPower(), af2);
        sleep(100);
        af2.setIntake(false);
        intakeR.setPower(0);
        intakeL.setPower(0);

        //Punem cubul sus

        af2.setOutputmotor(true, 1.25, true);
        am2.driveToAndTurnWithGyro(AutoMovement2.Directions.BACKWARD, (int) (saizecicm*2.25), 300, 1500, getPower(), af2, -65, 650);
        sleep(200);

        outputmotor.setPower(-0.75);
        sleep(650);
        outputmotor.setPower(0);

        af2.setOutputmotor(true, 2.5, false);

        am2.driveToAndTurnAndStrafeWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm*2.75), 300, 2300, getPower(), af2, 0, 445,800,900);

        af2.setOutputmotor(false, 0, false);
        sleep(200);

        //Luam cub 2
        af2.setIntake(true);
        am2.driveToWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm*0.38), 100, 400, getPower() * 0.35, af2);
        sleep(100);
        am2.turnWithGyro(AutoMovement2.Directions.BACKWARD, 5,(int) (saizecicm*0.63), 100, 150, getPower(), af2);
        sleep(100);
        af2.setIntake(false);
        intakeR.setPower(0);
        intakeL.setPower(0);

        //Punem cubul sus

        af2.setOutputmotor(true, 1.35, true);
        am2.driveToAndTurnWithGyro(AutoMovement2.Directions.BACKWARD, (int) (saizecicm*2.1), 400, 1600, getPower(), af2, -63, 400);
        sleep(200);

        outputmotor.setPower(-0.75);
        sleep(650);
        outputmotor.setPower(0);

        af2.setOutputmotor(true, 2.5, false);

        am2.driveToAndTurnAndStrafeWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm*2.9), 300, 2600, getPower(), af2, 0, 480,1000, 900);

        af2.setOutputmotor(false, 0, false);
        sleep(200);

        //Luam cub 3
        af2.setIntake(true);
        am2.driveToWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm*0.38), 100, 400, getPower() * 0.35, af2);
        sleep(100);
        am2.turnWithGyro(AutoMovement2.Directions.BACKWARD, 5,(int) (saizecicm*0.8), 100, 150, getPower(), af2);
        sleep(100);
        af2.setIntake(false);
        intakeR.setPower(0);
        intakeL.setPower(0);

        //Punem cubul sus

        af2.setOutputmotor(true, 1.35, true);
        am2.driveToAndTurnWithGyro(AutoMovement2.Directions.BACKWARD, (int) (saizecicm*2.138), 400, 1600, getPower(), af2, -61, 390);
        sleep(200);

        outputmotor.setPower(-0.75);
        sleep(650);
        outputmotor.setPower(0);

        af2.setOutputmotor(true, 2.5, false);

        am2.driveToAndTurnAndStrafeWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm*3), 300, 2300, getPower(), af2, 0, 530, 1000, 1000);

        af2.setOutputmotor(false, 0, false);
        sleep(200);

        //Luam cub4
        af2.setIntake(true);
        am2.driveToWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm*0.38), 100, 400, getPower() * 0.35, af2);
        sleep(100);
        am2.turnWithGyro(AutoMovement2.Directions.BACKWARD, 5,(int) (saizecicm*0.8), 100, 150, getPower(), af2);
        sleep(100);
        af2.setIntake(false);
        intakeR.setPower(0);
        intakeL.setPower(0);

        //Punem cubul sus

        af2.setOutputmotor(true, 1.35, true);
        am2.driveToAndTurnWithGyro(AutoMovement2.Directions.BACKWARD, (int) (saizecicm*2.375), 400, 1600, getPower(), af2, -61, 450);
        sleep(200);

        outputmotor.setPower(-0.75);
        sleep(650);
        outputmotor.setPower(0);

        af2.setOutputmotor(true, 2.5, false);

        am2.driveToAndTurnAndStrafeWithGyro(AutoMovement2.Directions.FORWARD, (int) (saizecicm*2.15), 300, 2300, getPower(), af2, 0, 490, 1000, 1000);

        af2.setOutputmotor(false, 0, false);
        sleep(200);

    }
}
