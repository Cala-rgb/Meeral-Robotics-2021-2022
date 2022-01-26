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
    private DcMotor intake1 = null;
    private DcMotor intake2 = null;
    int val,posinitial;
    double lasttimea=0.0,lasttimeb=0.0;
    boolean k=false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        intake1 = hardwareMap.get(DcMotor.class, "i1");
        intake2 = hardwareMap.get(DcMotor.class, "i2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.FORWARD);


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
        if(gamepad1.x && runtime.milliseconds()-lasttimea>500.0)
        {
            lasttimea=runtime.milliseconds();
            if(intake1.getPower()==0)
            {
                intake1.setPower(-0.5);
                intake2.setPower(0.5);
            }
            else{
                intake1.setPower(0);
                intake2.setPower(0);
            }
        }
        if(gamepad1.b && runtime.milliseconds()-lasttimeb>500.0)
        {
            lasttimeb=runtime.milliseconds();
            if(intake1.getPower()==0)
            {
                intake1.setPower(0.5);
                intake2.setPower(-0.5);
            }
            else
            {
                intake1.setPower(0);
                intake2.setPower(0);
            }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
