package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
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
public class testconstr extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo duckServo = null;
    private LED ledverde = null;
    private Movement mv;
    private IntakeAndOutput iao;
    BNO055IMU imu;
    private TeleOpAutoV1 toa;
    private TeleOpFuncV1 tof;
    private  void getEngines()
    {
        ledverde = hardwareMap.get(LED.class, "ledverde");
        duckServo = hardwareMap.get(CRServo.class, "duckServo");
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        getEngines();
        ledverde.enable(true);
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

       duckServo.setPower(gamepad1.left_stick_x);
        if(gamepad1.a)ledverde.enable(true);
        else if(gamepad1.b)ledverde.enable(false);
    }

    @Override
    public void stop() {
    }
}
