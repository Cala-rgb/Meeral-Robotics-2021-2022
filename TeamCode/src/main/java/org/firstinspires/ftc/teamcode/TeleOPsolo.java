package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    private ColorRangeSensor color= null;
    private double leftbump = 0;
    private double rightbump = 0;
    private double carut = 0;

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
        color = hardwareMap.get(ColorRangeSensor.class, "color");
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

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        getEngines();

        setDirections();

        mv = new Movement(fl,fr,bl,br);

        iao = new IntakeAndOutput(intakemotor1, intakemotor2, outputmotor,liftServoR, liftServoL, totemServo, duckServo, color, this);

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
        telemetry.addData("colorr", color.red());
        telemetry.addData("colorg", color.green());
        telemetry.addData("colorb", color.blue());
        telemetry.addData("dist", color.getDistance(DistanceUnit.CM));
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
        iao.verifyAll(rightbump,leftbump, gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.start, carut, runtime.milliseconds());
    }

    @Override
    public void stop() {
    }
}
