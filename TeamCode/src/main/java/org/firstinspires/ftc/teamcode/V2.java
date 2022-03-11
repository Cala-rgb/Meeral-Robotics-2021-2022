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
public class V2 extends OpMode {
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
    private double pow,bumpersteeringval,duckSpeed= -0.2,lasttimeb=0.0,lasttimex=0.0,cangle;
    private RevColorSensorV3 color= null, under = null;
    private VoltageSensor vs;

    BNO055IMU imu;

    private TeleOpAutoV1 toa;
    private TeleOpFuncV1 tof;

    private boolean autoMove = false;

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
        vs = hardwareMap.voltageSensor.iterator().next();
        under = hardwareMap.get(RevColorSensorV3.class, "under");
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

    private void autoMoveF() {
            tof.setTask(TeleOpFuncV1.Tasks.LEAVE_STORAGE);
            tof.setOutputmotor(true, 0.7, false);
            cangle = getAngle();
            toa.driveToAndTurnWithGyroWhen(TeleOpAutoV1.Directions.BACKWARD, 1581, 1300, (11.0 / vs.getVoltage()), tof, 65.7, true);
            double t = getRuntime();
            while (getRuntime() - t < 0.1)
                outputmotor.setPower(0.1);
            if (isPressed())
                return;
    }
    private void autoMoveB() {
            tof.setTask(TeleOpFuncV1.Tasks.NONE);
            tof.setOutputmotor(true, 1.4, true);
            toa.driveToAndTurnAndStrafeWithGyro(TeleOpAutoV1.Directions.FORWARD, (int) (1000*3), 300, 2300, (11.0/vs.getVoltage()), tof, cangle, 740,1100,1500);

            tof.setOutputmotor(false, 0, false);
            setDirections();
    }

    private boolean isPressed() {
        if(gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y || gamepad1.start || gamepad1.options || gamepad1.right_bumper || gamepad1.right_trigger!=0 || gamepad1.left_bumper || gamepad1.left_trigger!=0 || gamepad1.right_stick_y!=0 || gamepad1.right_stick_x!=0 || gamepad1.left_stick_x!=0 || gamepad1.left_stick_y!=0 || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_up || gamepad1.dpad_down)
            return true;
        return false;
    }

    private double getAngle() {
        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle.firstAngle;
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
        /*telemetry.addData("colorr", color.red());
        telemetry.addData("colorg", color.green());
        telemetry.addData("colorb", color.blue());
        telemetry.addData("dist", color.getDistance(DistanceUnit.CM));
        telemetry.update();*/
        if(gamepad1.a)
            pow =1;
        else if (gamepad1.y)
            pow = .25;
        else
            pow=0.58;
        if(gamepad2.b) {
            autoMove =! autoMove;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }
        if(gamepad1.b && getRuntime()-lasttimeb>0.5) {
            lasttimeb = getRuntime();
            autoMoveF();
        }
        if(gamepad1.x && getRuntime()-lasttimex>0.5) {
            lasttimex = getRuntime();
            autoMoveB();
        }
        mv.move(gamepad1.right_trigger, gamepad1.left_trigger, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, pow);
        if(gamepad1.right_bumper) {
            mv.break_func();
        }
        iao.verifyAll(gamepad2.right_trigger, gamepad2.left_trigger, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_right, gamepad2.dpad_left, gamepad1.start, gamepad2.left_stick_y, runtime.milliseconds());

    }

    @Override
    public void stop() {
    }
}
