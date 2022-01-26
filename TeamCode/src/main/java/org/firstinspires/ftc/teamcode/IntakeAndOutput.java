package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeAndOutput {


    //DcMotor caruselmotor;
    DcMotor intakemotor1;
    DcMotor intakemotor2;
    //DcMotor outputmotor;

    double lasttimex = 0.0, lasttimeb = 0.0;

    int posinitial=0;

    int val = 200;

    boolean k=false,apsatA=false;

    public IntakeAndOutput(DcMotor intakemotor1, DcMotor intakemotor2)
    {
        this.intakemotor1=intakemotor1;
        this.intakemotor2=intakemotor2;
    }

    public void verifyAll(boolean x, boolean b, boolean a, boolean y, double time)
    {
        turnOnIntake(x, b ,time);
        //turnOnOutput(a);
        //rotireCarusel(y);
    }

    void turnOnIntake(boolean x,boolean b ,double time)
    {
            if(x && time-lasttimex>500.0)
            {
                lasttimex=time;
                if(intakemotor1.getPower()==0)
                {
                    intakemotor1.setPower(-0.5);
                    intakemotor2.setPower(0.5);
                }
                else
                {
                    intakemotor1.setPower(0);
                    intakemotor2.setPower(0);
                }
            }
        if(b && time-lasttimeb>500.0)
        {
            lasttimeb=time;
            if(intakemotor1.getPower()==0)
            {
                intakemotor1.setPower(0.5);
                intakemotor2.setPower(-0.5);
            }
            else
            {
                intakemotor1.setPower(0);
                intakemotor2.setPower(0);
            }
        }
    }

    /*void turnOnOutput(boolean a)
    {
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
            if(apsatA==true)
            {
                outputmotor.setTargetPosition(posinitial);
                outputmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outputmotor.setPower(0.5);
                apsatA=false;
            }
        }
    }

    void rotireCarusel(boolean y)
    {
        if(y)
            caruselmotor.setPower(0.33);
    */

}
