package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class test extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor testmotor = null;
    int val,posinitial;
    double xval;
    boolean k=false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        xval = 1;
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        testmotor = hardwareMap.get(DcMotor.class, "testmotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        testmotor.setDirection(DcMotor.Direction.FORWARD);

        val =1000;
        posinitial=0;

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

        if(posinitial!=testmotor.getCurrentPosition()){
            k=true;
            xval -= 0.2;
        }
        if(testmotor.getCurrentPosition()==posinitial)
        {
            testmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(gamepad1.a)
            {
                posinitial = testmotor.getCurrentPosition();
            }
            while(gamepad1.a) {
                k=true;
                testmotor.setPower(1);
            }
            testmotor.setPower(0);
        }
        if(k==true)
        {
            testmotor.setTargetPosition(posinitial);
            testmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            testmotor.setPower(xval);
            k=false;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
