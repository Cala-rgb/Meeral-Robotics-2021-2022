package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class test extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor outputmotor = null, intakemotor1 = null, intakemotor2 = null;
    ColorSensor color;
    int val=-2000,posinitial;
    double lastTimeY=0.0;
    boolean apsatA = false,luat = false, k = false, apasatB = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    void turnOnOutput(boolean a)
    {
        if(a)
        {
            outputmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outputmotor.setPower(0.5);
        }
        outputmotor.setPower(0);
    }
    void luatObiect()
    {
        if(color.red()>100.0 && (color.red()>100.0 && color.green() > 100.0 && color.blue() > 100.0))
            luat =true;
    }

    void goToPos(boolean y, double time)
    {
        if(k==true && outputmotor.getCurrentPosition()!=outputmotor.getTargetPosition()){
            apasatB = false;
            int pos = outputmotor.getTargetPosition();
            outputmotor.setTargetPosition(pos);
            outputmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outputmotor.setPower(-1);
        }
        if(k==true && outputmotor.getCurrentPosition()==outputmotor.getTargetPosition())
        {
            outputmotor.setPower(0);
            luat = false;
            apasatB = true;
            k=false;
            telemetry.addData("ok", "ok");
            telemetry.update();
        }
        telemetry.addData("apasatB", apasatB);
        telemetry.addData("apasatB", outputmotor.getCurrentPosition());
        telemetry.update();
        if(luat && time-lastTimeY>500.0)
        {
            lastTimeY = time;
            if(!k && !apasatB)
            {
                k=true;
                apasatB=true;
                outputmotor.setTargetPosition(val);
                outputmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outputmotor.setPower(-1);
            }
            else if(apasatB)
            {
                outputmotor.setTargetPosition(0);
                outputmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outputmotor.setPower(-1);
                k = true;
                apasatB=false;
            }
        }
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

       // outputmotor = hardwareMap.get(DcMotor.class, "outputmotor");
        intakemotor1 = hardwareMap.get(DcMotor.class, "intakemotor1");
        intakemotor2 = hardwareMap.get(DcMotor.class, "intakemotor2");
        outputmotor = hardwareMap.get(DcMotor.class, "outputmotor");
        color = hardwareMap.get(ColorSensor.class, "color");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        //outputmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakemotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakemotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        outputmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //turnOnOutput(gamepad1.a);
        telemetry.addData("red", color.red());
        telemetry.addData("green", color.green());
        telemetry.addData("blue", color.blue());
        telemetry.update();
        luatObiect();
        goToPos(gamepad1.y, runtime.milliseconds());
        if (gamepad1.a) {
            intakemotor1.setPower(-0.821);
            intakemotor2.setPower(0.821);
        } else if (gamepad1.b){
            intakemotor1.setPower(0.821);
            intakemotor2.setPower(-0.821);
        } else {
            intakemotor1.setPower(0);
            intakemotor2.setPower(0);

        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
