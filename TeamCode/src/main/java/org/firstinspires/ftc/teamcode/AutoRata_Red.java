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
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

@Autonomous(name = "Autonom rata RED", group="Autonom competitie")
public class AutoRata_Red extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight, frontLeft, backRight, backLeft, intakeR, intakeL, outputmotor;
    private AutoMovement3 am3;
    private BNO055IMU imu;
    private CRServo duckServo1, duckServo2, lift;
    private Servo preloadedServo, totemServo;
    private OpenCvWebcam webcam;
    private RevColorSensorV3 color, under, under2;
    private TeamElementPipeline pipeline;
    private TeamElementPipeline.TeamElementPosition snapshotAnalysis = TeamElementPipeline.TeamElementPosition.LEFT;
    private ServoController sc;
    private AutoFunctionsV3 af3;
    int nouazecigrade = 610;
    int saizecicm = 1000;
    double uptime,distmulti=1.42;
    double startVoltage,minVoltage=20.0;
    VoltageSensor vs;

    private void getHardware() {

        duckServo1 = hardwareMap.get(CRServo.class, "duckServo1");
        duckServo2 = hardwareMap.get(CRServo.class, "duckServo2");
        lift= hardwareMap.get(CRServo.class, "lift");
        preloadedServo = hardwareMap.get(Servo.class, "preloadedservo");

        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft  = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        intakeR = hardwareMap.get(DcMotor.class, "intakeR");
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        outputmotor = hardwareMap.get(DcMotor.class, "outputmotor");
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        under = hardwareMap.get(RevColorSensorV3.class, "under");
        under2 = hardwareMap.get(RevColorSensorV3.class, "under2");

        vs = hardwareMap.voltageSensor.iterator().next();
        duckServo1.setDirection(DcMotorSimple.Direction.REVERSE);
        duckServo2.setDirection(DcMotorSimple.Direction.REVERSE);
        //?
    }

    private double getPower()
    {
        double voltage = vs.getVoltage();
        double power = 12.0 / voltage;
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

        telemetry.addData("ok","ok");
        telemetry.update();
        //sleep(3000);

        getHardware();

        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        am3 = new AutoMovement3(this, imu, frontRight, frontLeft, backRight, backLeft, vs);
        af3 = new AutoFunctionsV3(this, intakeR, intakeL, outputmotor, color, under, under2, lift);

        if(snapshotAnalysis == TeamElementPipeline.TeamElementPosition.LEFT) {
            uptime = 0;
        }
        else if(snapshotAnalysis == TeamElementPipeline.TeamElementPosition.CENTER){
            uptime = 0.9;
        }
        else {
            uptime = 3.2;
        }

        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        runtime.reset();

        //Setam servourile la pozitilor lor

        preloadedServo.setPosition(0.5);

        //Mergem in fata la shipping hub

        af3.setElevator(true, uptime, true);

        if(uptime == 3.2)
            distmulti = 1.425;

        am3.turnWithGyro(AutoMovement3.Directions.BACKWARD, 64, (int) (saizecicm * distmulti), 300, 1150, getPower() * 0.45, af3);

        if(uptime==3.2)
            sleep(300);

        //Lasam cubul preloaded
        lift.setPower(0);
        preloadedServo.setPosition(-0.2);
        sleep(500);
        preloadedServo.setPosition(0.5);
        sleep(200);

        if(snapshotAnalysis == TeamElementPipeline.TeamElementPosition.RIGHT)
            uptime-=2;
        else
            uptime=0;

        af3.setElevator(true,uptime,false);

        am3.driveToAndTurnWithGyro(AutoMovement3.Directions.FORWARD, 2180, 300, 1500, getPower()*0.2, af3, 0, 445);

        af3.setElevator(false,0,true);

        am3.strafeWithGyro(0.6, AutoMovement3.Directions.LEFT,getPower()*0.2,af3);

        duckServo2.setPower(0.35);
        sleep(3000);
        duckServo2.setPower(0);

        am3.strafeWithGyro(1.25, AutoMovement3.Directions.RIGHT,getPower(),af3);

        am3.driveToAndTurnWithGyro(AutoMovement3.Directions.FORWARD, 250, 100, 100, getPower()*0.2, af3, 0, 0);

        /*am3.driveToAndTurnAndStrafeWithGyro(AutoMovement3.Directions.FORWARD,(int) (saizecicm*1.4), 300, 1700, getPower(), af3, 125,0, 1000, 500, false);

        sleep(13000);

        am3.driveToAndTurnAndStrafeWithGyro(AutoMovement3.Directions.FORWARD,(int) (saizecicm*3), 300, 1700, getPower(), af3, 180,0, 200, 2000, false);*/

    }
}
