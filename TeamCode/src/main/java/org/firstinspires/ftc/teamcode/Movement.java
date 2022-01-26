package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Movement {

    private DcMotor fl,fr,bl,br;

    public Movement(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br)
    {
        this.fl=fl;
        this.fr=fr;
        this.bl=bl;
        this.br=br;
    }

    void move(double forwardpower, double backwardpower, double steer, double strafe, double rsthr,double pow)
    {
        double throttle = -forwardpower+backwardpower+rsthr;
        fl.setPower((throttle-steer-strafe)*pow);
        fr.setPower((throttle+steer+strafe)*pow);
        bl.setPower((throttle-steer+strafe)*pow);
        br.setPower((throttle+steer-strafe)*pow);
    }

    void bumbersteering(double pow)
    {
        fl.setPower(-pow);
        fr.setPower(pow);
        bl.setPower(-pow);
        br.setPower(pow);
    }

}
