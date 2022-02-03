package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class testcolor extends OpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor color;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        color = hardwareMap.get(ColorSensor.class, "color");

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
        telemetry.addData("red", color.red());
        telemetry.addData("red", color.green());
        telemetry.addData("red", color.blue());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
