package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class TeleOPsolo extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private CRServo duckServo = null;
    private CRServo liftServoR = null;
    private CRServo liftServoL = null;
    private Servo totemServo = null;
    private Servo preloadedServo = null;

    private DcMotor intakemotor1 = null;
    private DcMotor intakemotor2 = null;
    private DcMotor outputmotor = null;
    private Movement mv;
    private IntakeAndOutput iao;
    private double pow,bumpersteeringval,duckSpeed= -0.2;
    private RevColorSensorV3 color= null, under = null, under2 = null;
    private double leftbump = 0;
    private double rightbump = 0;
    private double carut = 0;
    private VoltageSensor vs;
    private TeleOpAutoV1 toa;
    private TeleOpFuncV1 tof;

    BNO055IMU imu;

    private  void getEngines()
    {
        duckServo = hardwareMap.get(CRServo.class, "duckServo");
        liftServoL = hardwareMap.get(CRServo.class, "lift2");
        liftServoR = hardwareMap.get(CRServo.class, "lift1");
        totemServo = hardwareMap.get(Servo.class, "totemservo");
        preloadedServo = hardwareMap.get(Servo.class, "preloadedservo");

        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        intakemotor1 = hardwareMap.get(DcMotor.class, "intakeR");
        intakemotor2 = hardwareMap.get(DcMotor.class, "intakeL");
        outputmotor = hardwareMap.get(DcMotor.class, "outputmotor");
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        under = hardwareMap.get(RevColorSensorV3.class, "under");
        vs = hardwareMap.voltageSensor.iterator().next();
        under = hardwareMap.get(RevColorSensorV3.class, "under");
        under2 = hardwareMap.get(RevColorSensorV3.class, "under2");
    }

    private void setDirections()
    {
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        intakemotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakemotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        outputmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outputmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outputmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getAngle() {
        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle.firstAngle;
    }

    private void autoMove() {
        tof.setTask(TeleOpFuncV1.Tasks.LEAVE_STORAGE);
        tof.setOutputmotor(true, 0.8, false);
        double cangle = getAngle();
        toa.driveToAndTurnWithGyroWhen(TeleOpAutoV1.Directions.BACKWARD,1733, 1450,(11.0/vs.getVoltage()), tof, 63.21, true);
        double time = getRuntime();
        if(gamepad1.b)
            return ;
        while(getRuntime()-time<0.6)
            outputmotor.setPower(0.85);
        outputmotor.setPower(0);
        tof.setOutputmotor(true, 2.5, true);
        if(gamepad1.b)
            return ;
        toa.driveToAndTurnAndStrafeWithGyro(TeleOpAutoV1.Directions.FORWARD, (int) (1000*3.05), 300, 2300, (11.0/vs.getVoltage()), tof, cangle, 710,1000,1100);

        tof.setOutputmotor(false, 0, false);
        setDirections();
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        getEngines();

        setDirections();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        mv = new Movement(fl,fr,bl,br,outputmotor,imu,vs,under,this);

        iao = new IntakeAndOutput(intakemotor1, intakemotor2, outputmotor,liftServoR, liftServoL, totemServo, duckServo, color, this);

        toa = new TeleOpAutoV1(this, imu, fr, fl, br, bl, vs);

        tof = new TeleOpFuncV1(this, intakemotor1, intakemotor2,outputmotor, color, under, duckServo, liftServoR, liftServoL);

        bumpersteeringval = 0.25;

        pow=0.5;

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("colorr", under.red());
        telemetry.addData("colorg", under.green());
        telemetry.addData("colorb", under.blue());
        telemetry.addData("alpha", under.alpha());
        telemetry.addData("lait", under.getLightDetected());
        telemetry.addData("dist", under.getDistance(DistanceUnit.CM));
        telemetry.update();
        if(gamepad1.a)
            pow =1;
        else if (gamepad1.y)
            pow = .25;
        else
            pow=0.5;
        if(gamepad1.right_bumper) {
            rightbump = 1;
            leftbump = 0;
        }
        else if(gamepad1.left_bumper) {
            rightbump = 0;
            leftbump = 1;
        }
        else {
            rightbump = 0;
            leftbump = 0;
        }
        if(gamepad1.b){
            carut = -1;
        }
        else if(gamepad1.x){
            carut = 1;
        }
        else {
            carut = 0;
        }
        mv.move(gamepad1.right_trigger, gamepad1.left_trigger, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, pow);
        if(gamepad1.back) {
            autoMove();
        }
        iao.verifyAll(rightbump,leftbump, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.start, gamepad1.left_stick_y, runtime.milliseconds());
    }

    @Override
    public void stop() {
    }
}
