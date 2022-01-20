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

    void move(double forwardpower, double backwardpower, double steer, double strafe, double rsthr)
    {
        double throttle = forwardpower-backwardpower+rsthr;
        fl.setPower(throttle-steer-strafe);
        fr.setPower(throttle+steer+strafe);
        bl.setPower(throttle-steer+strafe);
        br.setPower(throttle+steer-strafe);
    }

}
