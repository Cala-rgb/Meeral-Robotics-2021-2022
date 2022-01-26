package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class V1 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    //private DcMotor caruselmotor = null;
    private DcMotor intakemotor1 = null;
    private DcMotor intakemotor2 = null;
    //private DcMotor outputmotor = null;
    private Movement mv;
    private IntakeAndOutput iao;
    private double pow,bumpersteeringval;

    double metripozb;

    private  void getEngines()
    {
        fl  = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        //caruselmotor = hardwareMap.get(DcMotor.class, "caruselmotor");
        intakemotor1 = hardwareMap.get(DcMotor.class, "intakeR");
        intakemotor2 = hardwareMap.get(DcMotor.class, "intakeL");
        //outputmotor = hardwareMap.get(DcMotor.class, "outputmotor");
    }

    private void setDirections()
    {
        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        //caruselmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakemotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakemotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        //outputmotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        getEngines();

        setDirections();

        mv = new Movement(fl,fr,bl,br);

        iao = new IntakeAndOutput(intakemotor1, intakemotor2);

        //iao.convertmtorpm(metripozb);

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
        if(gamepad1.dpad_down)
            pow = 0.5;
        if(gamepad1.dpad_up)
            pow = 1;
        if(gamepad1.a)
            pow =1;
        else
            pow=0.5;
        if(gamepad1.right_bumper)
            mv.bumbersteering(bumpersteeringval);
        if(gamepad1.left_bumper)
            mv.bumbersteering(-bumpersteeringval);
        mv.move(gamepad1.right_trigger, gamepad1.left_trigger, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, pow);
        iao.verifyAll(gamepad1.x,gamepad1.b,gamepad1.a,gamepad1.y, runtime.milliseconds());
    }

    @Override
    public void stop() {
    }
}
