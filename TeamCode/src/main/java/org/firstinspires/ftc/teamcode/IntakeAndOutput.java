package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeAndOutput {


    DcMotor caruselmotor;
    DcMotor intakemotor1;
    DcMotor intakemotor2;
    DcMotor outputmotor;

    double lasttimex = 0.0, lasttimeb = 0.0;

    int posinitial=0;

    int val = 200;

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
            if(x && time-lasttimex>500.0)
            {
                lasttimex=time;
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
            outputmotor.setTargetPosition(val);
            k=true;
        }
        else
        {
            outputmotor.setTargetPosition(0);
            k=false;
        }
        outputmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outputmotor.setPower(0.2);

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
        if(b && time-lasttimeb>500.0)
        {
            lasttimeb = time;
            deplaseazaxmetri();
        }
        if(a)
        {
            posinitial = outputmotor.getCurrentPosition();
            if(posinitial!=outputmotor.getCurrentPosition())apsatA=true;
            if(outputmotor.getCurrentPosition()==posinitial)
            {
                outputmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(a)
                {
                    posinitial = outputmotor.getCurrentPosition();
                }
                while(a) {
                    k=true;
                    outputmotor.setPower(1);
                }
                outputmotor.setPower(0);
            }
            if(apsatA)
            {
                outputmotor.setTargetPosition(posinitial);
                outputmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outputmotor.setPower(1);
                apsatA=false;
            }
        }
    }

    void rotireCarusel(boolean start)
    {
        if(start)
            caruselmotor.setPower(0.33);
    }

}
