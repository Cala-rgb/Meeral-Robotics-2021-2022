package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeAndOutput {


    DcMotor caruselmotor;
    DcMotor intakemotor1;
    DcMotor intakemotor2;
    DcMotor outputmotor;

    double lasttime = 0.0;

    boolean k=false,apsatA=false;

    public IntakeAndOutput(DcMotor caruselmotor, DcMotor intakemotor1, DcMotor intakemotor2, DcMotor outputmotor)
    {
        this.caruselmotor=caruselmotor;
        this.intakemotor1=intakemotor1;
        this.intakemotor2=intakemotor2;
        this.outputmotor=outputmotor;
    }

    public void verifyAll(boolean x, boolean b, boolean a, boolean start, double time)
    {
        turnOnIntake(x, time);
        turnOnOutput(a, b, time);
        rotireCarusel(start);
    }

    void turnOnIntake(boolean x, double time)
    {
            if(x && time-lasttime>500.0)
            {
                lasttime=time;
                if(intakemotor1.getPower()==0)
                {
                    intakemotor1.setPower(1);
                    intakemotor2.setPower(-1);
                }
                else
                {
                    intakemotor1.setPower(0);
                    intakemotor2.setPower(0);
                }
            }
    }

    public void convertmtorpm(double metri)
    {
        //convertim metri in rpm
    }

    void deplaseazaxmetri()
    {
        if(k==false)
        {
            //invarte motoru x metri in fata
            k=true;
        }
        else
        {
            //invarte motoru x metri in spate
            k=false;
        }

    }

    void countrotatii()
    {
        //numaram rotatile
    }

    void deplasaremotora()
    {
        //deplasam motorul cat timp e apasat a
        outputmotor.setPower(1);
        countrotatii();
    }

    void turnOnOutput(boolean a, boolean b, double time)
    {
        if(b && time-lasttime>500.0)
        {
            lasttime = time;
            deplaseazaxmetri();
        }
        while(a)
        {
            apsatA = true;
            deplasaremotora();
        }
        if(apsatA)
        {
            //intoarcete inappi
            apsatA=false;
        }
    }

    void rotireCarusel(boolean start)
    {
        if(start)
            caruselmotor.setPower(0.33);
    }

}
