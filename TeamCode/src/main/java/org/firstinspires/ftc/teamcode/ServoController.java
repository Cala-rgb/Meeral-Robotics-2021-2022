package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ServoController {
    private CRServo servo1, servo2;
    private DcMotor outputmotor;
    public ServoController(CRServo servo1, CRServo servo2, DcMotor outputmotor) {
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.outputmotor = outputmotor;
    }
    public void setPowerToServos(double power, boolean ok) {
        if(ok)
        {
            servo1.setPower(power);
            servo2.setPower(-power);
        }
        else
        {
            outputmotor.setPower(power);
        }
    }
}
