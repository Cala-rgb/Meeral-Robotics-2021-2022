package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

    @TeleOp
    public class clasalupandele extends OpMode {
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


        boolean thread_run=true; /*Set to false to stop the thread i.e. when your opmode is ending */
        double rpm_gate_time=250; /*How long to wait (mS) between encoder samples - trade off between update rate and precision*/
        double LRPM,RRPM; /*Left motor RPM, Right motor RPM*/
        @Override
        public void init(){
            /* ..... */
            ElapsedTime tm = new ElapsedTime();
            new Thread(new Runnable() {
                @Override
                public void run() {
                    double sms;
                    while (thread_run) {
                        /*left and right are dcMotor instances*/
                        double last_left_encoder = fl.getCurrentPosition(); /*Get first sample*/
                        double last_right_encoder = fr.getCurrentPosition();
                        sms = tm.milliseconds();
                        while(tm.milliseconds()-sms < rpm_gate_time){} /*Wait rpm_gate_time mS*/
                        double delta_l = fl.getCurrentPosition() - last_left_encoder; /*Get second sample, subtract first sample to get change*/
                        double delta_r = fr.getCurrentPosition() - last_right_encoder;
                        double factor = ((1000/rpm_gate_time)*60)/1120; /*Compute factor to convert encoder ticks per gate window to RPM (assumes 1120 ticks/rev)*/
                        double RPM = delta_l * factor; /*Calculate the RPM for the left motor*/
                        if(Math.abs(RPM) < 400){
                            /*If we get an absurdly high RPM, the it may be encoder jitter or the encoder was reset mid reading; keep the last reading instead*/
                            LRPM = -RPM; /*Store the calculated RPM in the LRPM variable*/
                        }
                        RPM = delta_r * factor; /*^ Ditto for the right motor*/
                        if(Math.abs(RPM) < 400){
                            RRPM = -RPM;
                        }
                    }
                }
            }).start();
            /* ..... */

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
            telemetry.addData("Motor RPM: ", "%.f, %.f", LRPM, RRPM); //The last measured/computed RPM value will always be available in the LRPM and RRPM global variables
            telemetry.update();
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
