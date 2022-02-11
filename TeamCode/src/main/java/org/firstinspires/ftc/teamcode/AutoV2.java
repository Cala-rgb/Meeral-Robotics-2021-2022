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

@Autonomous(name="Autonom version 2", group="Autonom bomba")
public class AutoV2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight, frontLeft, backRight, backLeft, intakeR, intakeL, outputmotor;
    private AutoMovement am;
    private BNO055IMU imu;
    private CRServo duckServo, liftServoR, liftServoL;
    private Servo preloadedServo, totemServo;
    private OpenCvWebcam webcam;
    private ColorRangeSensor color;
    private TeamElementPipeline pipeline;
    private TeamElementPipeline.TeamElementPosition snapshotAnalysis = TeamElementPipeline.TeamElementPosition.LEFT;
    private ServoController sc;
    private AutoFunc af;
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

        double upTime = 0;

        if(snapshotAnalysis == TeamElementPipeline.TeamElementPosition.LEFT) {
            upTime = 0;
        }
        else if(snapshotAnalysis == TeamElementPipeline.TeamElementPosition.CENTER){
            upTime = 0.75;
        }
        else {
            upTime = 3;
        }

        getHardware();
        sc = new ServoController(liftServoR, liftServoL, outputmotor);
        am = new AutoMovement(this, imu, frontRight, frontLeft, backRight, backLeft, vs);
        af = new AutoFunc(intakeR, intakeL, color, duckServo, this, frontRight, frontLeft, backRight, backLeft, am, outputmotor);

        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        runtime.reset();

        double voltage = vs.getVoltage();
        double power= 11/voltage;

        preloadedServo.setPosition(0.8);

        //am.driveToWithGyro(AutoMovement.Directions.FORWARD, 2000, 300, 700, power);
        //am.driveTo(AutoMovement.Directions.ROTATE_LEFT, nouazecigrade, 300, 700, power*0.8);

        /*am.driveToWithGyro(AutoMovement.Directions.FORWARD, 2000, 500, 1500, power);
        sleep(2000);
        //am.driveToWithGyro(AutoMovement.Directions.RIGHT, 1500, 400, 1100, power);
        //sleep(2000);
        am.driveTo(AutoMovement.Directions.ROTATE_RIGHT, nouazecigrade, 100, 510, power*0.8);
        sleep(2000);
        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, saizecicm, 300, 700, power);
        sleep(2000);*/

        am.strafeWithGyro(0.3, AutoMovement.Directions.RIGHT, power);
        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm*0.9), 300, 600, power, upTime, sc, true, true);
        af.duck();
        liftServoR.setPower(0);
        liftServoL.setPower(0);
        /*am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm * 2.1), 300, 700, power);
        sleep(100);
        am.driveTo(AutoMovement.Directions.ROTATE_LEFT, nouazecigrade, 300, 510, power*0.8);
        sleep(100);
        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm*0.65), 300, 510, power);
        preloadedServo.setPosition(0);
        sleep(300);*/
        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm ), 300, 700, power,intakeL, intakeR, color, power);
        am.turnB(55, AutoMovement.Directions.BACKWARD, (int) (saizecicm * 0.5), 300, 400, power);
        preloadedServo.setPosition(0);
        sleep(300);
        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm * 0.1), 100, 100, power);
        am.turn(135, AutoMovement.Directions.FORWARD, (int) (saizecicm * 2), 300, 1700, power);
        //am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm), 300, 700, power);
        //am.driveTo(AutoMovement.Directions.ROTATE_LEFT, nouazecigrade, 300, 510, power*0.8);
        //am.strafeWithGyro(0.3, AutoMovement.Directions.RIGHT, power);
        am.turn(170, AutoMovement.Directions.FORWARD, (int) (saizecicm * 2.4), 300, 2100, power);
        //am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm*2), 300, 1700, power);
        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm*0.2), 200, 400, power*0.2);
        am.driveAndIntake(AutoMovement.Directions.FORWARD, (int) (saizecicm*0.6), 100, 450, power*0.35, intakeL, intakeR, color, power);
        //am.turnB(179, AutoMovement.Directions.BACKWARD, (int) (saizecicm * 0.35), 100, 250, power, 0.5, sc, false, false);
        sleep(200);
        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm * 1.3), 200, 1000, power, 1, sc, true, false);
        am.turnB(115, AutoMovement.Directions.BACKWARD, (int) (saizecicm*2), 300, 1700, power, 1.25, sc, false, true);
        outputmotor.setPower(-1);
        sleep(1000);
        outputmotor.setPower(0);
        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm * 1.1), 300, 800, power, 1.5, sc, false, false);
        sleep(100);
        am.turn(170, AutoMovement.Directions.FORWARD, (int) (saizecicm * 2.4), 300, 2400, power, 0.5, sc, false, false);
        //primu cub
        am.driveAndIntake(AutoMovement.Directions.FORWARD, (int) (saizecicm*0.6), 100, 450, power*0.35, intakeL, intakeR, color, power);
        //am.turnB(179, AutoMovement.Directions.BACKWARD, (int) (saizecicm * 0.35), 100, 250, power, 0.5, sc, false, false);
        sleep(200);

        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm * 1.3), 200, 1000, power, 1, sc, true, false);
        am.turnB(115, AutoMovement.Directions.BACKWARD, (int) (saizecicm*2), 300, 1800, power, 1.25, sc, false, true);
        outputmotor.setPower(-1);
        sleep(1000);
        outputmotor.setPower(0);
        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm * 1.15), 300, 850, power, 1.5, sc, false, false);
        sleep(100);
        am.turn(170, AutoMovement.Directions.FORWARD, (int) (saizecicm * 2.4), 300, 2400, power, 0.5, sc, false, false);
        //cub2
        am.driveAndIntake(AutoMovement.Directions.FORWARD, (int) (saizecicm*0.6), 100, 450, power*0.35, intakeL, intakeR, color, power);
        //am.turnB(179, AutoMovement.Directions.BACKWARD, (int) (saizecicm * 0.35), 100, 250, power, 0.5, sc, false, false);
        sleep(200);
        am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm * 1.3), 200, 1000, power, 1, sc, true, false);
        am.turnB(115, AutoMovement.Directions.BACKWARD, (int) (saizecicm*2), 300, 1800, power, 1.25, sc, false, true);
        outputmotor.setPower(-1);
        sleep(1000);
        outputmotor.setPower(0);
        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm * 1.2), 300, 900, power, 1.5, sc, false, false);
        sleep(100);
        am.turn(170, AutoMovement.Directions.FORWARD, (int) (saizecicm * 2.4), 300, 2400, power, 0.5, sc, false, false);
        /*am.driveBToWithGyro(AutoMovement.Directions.BACKWARD, (int) (saizecicm*0.8), 300, 700, power);
        outputmotor.setPower(-power);
        sleep(2000);
        outputmotor.setPower(0);
        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm*0.85), 300, 700, power);
        sleep(100);
        am.driveTo(AutoMovement.Directions.ROTATE_LEFT, (int) (nouazecigrade * 0.5), 100, 350, power*0.8);
        sleep(100);
        am.strafeWithGyro(0.6, AutoMovement.Directions.RIGHT, power);
        sleep(100);
        am.driveToWithGyro(AutoMovement.Directions.FORWARD, (int) (saizecicm*2), 300, 1700, power);*/




        /*am.driveToWithGyro(2.5, AutoMovement.Directions.FORWARD, power);
        intakein(power);
        am.driveToWithGyro(2.7, AutoMovement.Directions.BACKWARD, power, 2000, sc, false);
        am.rotateTo(90, power*0.8);
        am.driveToWithGyro(1.5, AutoMovement.Directions.FORWARD, power);
        af.output();
        am.rotateTo(180, power*0.8);
        am.strafeWithGyro(1.5, AutoMovement.Directions.RIGHT, power);
        am.driveToWithGyro(2, AutoMovement.Directions.FORWARD, power);*/
    }
}
