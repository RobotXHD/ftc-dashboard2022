/*

UltimateGoal01

Holonomic Drive

* sqrt transfer function
* normalized power

2020.12.06

*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Disabled
public class FreitFrenzi extends OpMode{
    //private Gyroscope imu;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotorEx arm;
    private DcMotorEx arm2;
    private DcMotorEx DJL;
    private DcMotorEx DJR;
    private Servo loader;
    private Servo cupa;
    private Servo claw_left;
    private Servo claw_right;
    double sm = 1;
    double poz = 0;
    double gpoz = 0.5;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    public DistanceSensor DistGheara;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    double lastTime;
    float right_stick2;
    float right_stick1;
    boolean v = true,ok1,ok2,ok3,ok4,ok5,ok6;
    boolean FirstTime = true;
    boolean inchis = false, inchis2 = false;
    boolean overpower = true;
    boolean permisie = true;
    boolean stopDJ = false;
    boolean tru=false;
    private boolean stop;
    int okGrip = 1;
    //long VoltageSensor;
    public ElapsedTime timer = new ElapsedTime();
    double timeLimit = 0.25;
    int loaderState = -1;


    public void init() {
        motorBL = hardwareMap.get(DcMotor.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotor.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotor.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotor.class, "motorFR"); // Motor Front-Right
        arm     = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm2    = (DcMotorEx) hardwareMap.dcMotor.get("arm2");
        DJL     = (DcMotorEx) hardwareMap.dcMotor.get("DJL");
        DJR     = (DcMotorEx) hardwareMap.dcMotor.get("DJR");
        claw_left         = hardwareMap.servo.get("loader");
        claw_right        = hardwareMap.servo.get("cupa");
        DistGheara = hardwareMap.get(DistanceSensor.class, "DistGheara");

        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);


        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DJL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DJR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //arm.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //arm.setTargetPosition(0);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Resseting", "Encoders");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        // run until the end of the match (driver presses STOP)

        claw_left.setPosition(0.55);
        claw_right.setPosition(0.45);
    }
    @Override
    public void start(){
        Chassis.start();
        Systems.start();
        Sensors.start();
    }
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            while(!stop) {
                if(gamepad2.left_bumper) {
                    ok1 = false;
                    ok2 = false;
                    ok3 = false;
                    ok4 = false;
                    ok5 = false;
                    ok6 = false;
                }
                if(gamepad2.right_bumper){
                    ok1 = true;
                    ok2 = true;
                    ok3 = true;
                    ok4 = true;
                    ok5 = true;
                    ok6 = true;
                }
                if(gamepad1.left_trigger!=0) {
                    ok1 = false;
                    ok2 = false;
                    ok3 = false;
                    ok4 = false;
                    ok5 = false;
                    ok6 = false;
                }
                if(gamepad1.right_trigger!=0){
                    ok1 = true;
                    ok2 = true;
                    ok3 = true;
                    ok4 = true;
                    ok5 = true;
                    ok6 = true;
                }
                tru = true;

                y  = -gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x * 1.5;
                rx = gamepad1.right_stick_x;


                pmotorFL = y + x + rx;
                pmotorBL = y - x + rx;
                pmotorBR = y + x - rx;
                pmotorFR = y - x - rx;

                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }
                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }

                //SLOW-MOTION
                if (gamepad1.left_bumper) {
                    sm = 2;
                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    //arm.setPower(poz/sm);
                } else {
                    //SLOWER-MOTION
                    if (gamepad1.right_bumper) {
                        sm = 5;
                        POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    } else {
                        sm = 0.5;
                        POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    }
                }
            }
        }
    });
    private final Thread Sensors = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!stop){
                //CLAPETA INCHISA
                if (gamepad1.a) {
                    inchis = true;
                    claw_left.setPosition(0.42);//1.0
                    claw_right.setPosition(0.58);
                }

                //CLAPETA INTRE
                if (gamepad1.dpad_up) {
                    inchis = true;
                    //loader.setPosition(0.15);//0.5   0.15
                }

                //CLAPETA DESCHISA
                if (gamepad1.y) {
                    inchis = false;
                    claw_left.setPosition(0.55);//0.0   0.0
                    claw_right.setPosition(0.45);
                }
                if(DistGheara.getDistance(DistanceUnit.CM) >= 8.5 && inchis == false) {
                    inchis2 = false;
                }
                else if(DistGheara.getDistance(DistanceUnit.CM) < 8.5 && inchis2 == false) {
                    claw_left.setPosition(0.42);//1.0
                    claw_right.setPosition(0.58);
                    inchis2 = true;
                    inchis = true;
                }
            }
        }
    });
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
            /*
        if(gamepad1.a)
        {
            motorFL.setPower(-0.08);
            motorFR.setPower(0.08);
            motorBL.setPower(-0.08);
            motorBR.setPower(0.08);
        }
        */
/*
                if (gamepad1.left_trigger > 0.0) {
                    grabber_left.setPosition(0.1);//0.1
                } else {
                    grabber_left.setPosition(0.8);//0.8
                }

                if (gamepad1.right_trigger > 0.0) {
                    grabber_right.setPosition(0.9);//0.9
                } else {
                    grabber_right.setPosition(0.2);//0.2
                }
 */



/*
                if (gamepad2.left_trigger > 0.0) {
                    grabber_left.setPosition(0.1);//0.1
                } else {
                    grabber_left.setPosition(0.8);//0.8
                }

                if (gamepad2.right_trigger > 0.0) {
                    grabber_right.setPosition(0.9);//0.9
                } else {
                    grabber_right.setPosition(0.2);//0.2
                }
 */

                /*
                grabber_left.setPosition(-gamepad1.left_trigger);
                grabber_right.setPosition(gamepad1.right_trigger);
                */

/*
                //ARM-UP
                if (gamepad1.dpad_up) {
                    while (gamepad1.dpad_up) {
                        poz--;
                        arm.setPower(poz);
                    }
                }
                arm.setPower(0);
                poz = 0;


                //ARM-DOWN
                if (gamepad1.dpad_down) {
                    while (gamepad1.dpad_down) {
                        poz++;
                        arm.setPower(poz);
                    }
                }
                arm.setPower(0);
                poz = 0;

 */

                //DJL
                if (gamepad1.x) {
                    if (stopDJ == true)
                        stopDJ = false;
                    DJL.setPower(0.6);
                } else DJL.setPower(0);
                stopDJ = true;

                DJL.setPower(gamepad2.left_stick_y);
                DJR.setPower(gamepad2.left_stick_y);



                if (gamepad1.back)
                    arm.setMode(DcMotor.RunMode.RESET_ENCODERS);
                if (gamepad2.back)
                    arm.setMode(DcMotor.RunMode.RESET_ENCODERS);

                /*
                if (arm.getCurrentPosition() >= 250)
                {
                    arm.setPower(gamepad1.left_stick_y / 10);
                }
                 */

                right_stick1 = gamepad1.right_stick_y;
                if (arm.getCurrentPosition() <= 0) {
                    if (right_stick1 < 0) {
                        arm.setPower(0);
                        arm2.setPower(0);
                    }
                    else {
                        arm.setPower(right_stick1);
                        arm2.setPower(-right_stick1);
                    }
                }
                else{
                    arm.setPower(right_stick1);
                    arm2.setPower(-right_stick1);
                }
                        /*                  }
                    if (arm.getCurrentPosition() >= 250) {
                        arm.setPower(gamepad1.right_stick_y / 10);
                        arm2.setPower(-gamepad1.right_stick_y / 10);
                    }
 */







                //gamepad2
                right_stick2 = gamepad2.right_stick_y;
                if (arm.getCurrentPosition() <= 0) {
                    if (right_stick2 < 0) {
                        arm.setPower(0);
                        arm2.setPower(0);
                    }
                    else {
                        arm.setPower(-right_stick2);
                        arm2.setPower(right_stick2);
                    }
                }
                else{
                    arm.setPower(-right_stick2);
                    arm2.setPower(right_stick2);
                }



/*
                if (gamepad2.x) {
                    gpoz=gpoz + 0.001;
                    cupa.setPosition(gpoz);//0.67
                }

                if (gamepad2.b) {
                    gpoz=gpoz - 0.001;
                    cupa.setPosition(gpoz);//0.27
                }
 */

                /*
                neutru: 0.69
                level1: 0.52
                level2: 0.40
                level3: 0.27
                 */




                //CLAPETA INCHISA
                if (gamepad2.a) {
                    inchis = true;
                    claw_left.setPosition(0.42);
                    claw_right.setPosition(0.58);
                }

                //CLAPETA DESCHISA
                if (gamepad2.y) {
                    inchis = false;
                    claw_left.setPosition(0.55);
                    claw_right.setPosition(0.45);
                }
                /*
                if(gamepad2.b) {
                arm.setTargetPosition(200);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);
                }
                 */



/*
        BUN
        if (loader.getPosition() == 1.0 && arm.getCurrentPosition() == 100 ) //90
        {
            loader.setPosition(0.0);
            inchis = false;

*/
                /*
                if (overpower = true) {
                    if (inchis == true) {
                        if (loader.getPosition() == 1.0 && arm.getCurrentPosition() <= 430 && arm.getCurrentPosition() >= 420) //90
                        {
                            inchis = false;
                            loader.setPosition(0.0);

                        }
                    }
                }
                */

        /*
        if (loader.getPosition() == 1.0 && arm.getCurrentPosition() >= 300)
        {
            loader.setPosition(0.0);
            inchis = false;
        }
        */

        /*
        if (permisie == true || permisie == false) {
            if (gamepad1.right_stick_y > 0 || gamepad1.right_stick_y < 0)
                permisie = true;
                arm.setPower(gamepad1.right_stick_y / 1.5);
    }
         */

        /*
        if (permisie == true) {
            if (gamepad1.x) {
                permisie = false;
                arm.setMode(DcMotor.RunMode.RESET_ENCODERS);
            }
        }
         */

                //bun
        /*
        if (gamepad1.x)
            if (gamepad1.right_stick_y < 0)
                arm.setPower(0);
         */


                /*
                if (gamepad1.right_stick_y > 0 || gamepad1.right_stick_y < 0) {
                    arm.setPower(gamepad1.right_stick_y);
                    /*
                    if (arm.getCurrentPosition() < 250)
                    {
                        arm.setPower(gamepad1.right_stick_y / 10);
                    }
                     */


                if (gamepad1.b) {
                    ok1=true;
                    ok2=false;
                    ok3=false;
                    ok4=false;
                    ok5=false;
                    ok6=false;
                    //while(arm.getCurrentPosition() != 160)
                    //arm.setTargetPosition(160);
                    //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //arm.setPower(1.0);
                    arm.setTargetPosition(450);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.8);
                    arm2.setPower(-0.8);
                    while (arm.isBusy()&&ok1==true);
                    arm.setPower(0);
                    arm2.setPower(0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    ok2=true;
                    ok3=true;
                    ok4=true;
                    ok5=true;
                    ok6=true;
                }

                if (gamepad1.dpad_left) {
                    ok1=false;
                    ok2=true;
                    ok3=false;
                    ok4=false;
                    ok5=false;
                    ok6=false;
                    //while(arm.getCurrentPosition() != 160)
                    //arm.setTargetPosition(160);
                    //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //arm.setPower(1.0);
                    arm.setTargetPosition(0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1.0);
                    arm2.setPower(1.0);
                    while (arm.isBusy()&&ok2==true) ;
                    arm.setPower(0);
                    arm2.setPower(0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastTime = System.currentTimeMillis();
                    ok1=true;
                    ok3=true;
                    ok4=true;
                    ok5=true;
                    ok6=true;
                }

                if (gamepad1.dpad_right) {
                    ok1=false;
                    ok2=false;
                    ok3=true;
                    ok4=false;
                    ok5=false;
                    ok6=false;
                    //while(arm.getCurrentPosition() != 160)
                    //arm.setTargetPosition(160);
                    //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //arm.setPower(1.0);
                    arm.setTargetPosition(580);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1.0);
                    arm2.setPower(-1.0);
                    while (arm.isBusy()&&ok3==true) ;
                    arm.setPower(0);
                    arm2.setPower(0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastTime = System.currentTimeMillis();
                    ok1=true;
                    ok2=true;
                    ok4=true;
                    ok5=true;
                    ok6=true;
                }

                if (gamepad1.dpad_up) {
                    ok1=false;
                    ok2=false;
                    ok3=false;
                    ok4=true;
                    ok5=false;
                    ok6=false;
                    //while(arm.getCurrentPosition() != 160)
                    //arm.setTargetPosition(160);
                    //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //arm.setPower(1.0);
                    arm.setTargetPosition(430);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.65);
                    arm2.setPower(-0.65);
                    while (arm.isBusy()&&ok4==true);
                    arm.setPower(0);
                    arm2.setPower(0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastTime = System.currentTimeMillis();
                    ok1=true;
                    ok2=true;
                    ok3=true;
                    ok5=true;
                    ok6=true;
                }


                //gamepad2
                if (gamepad2.dpad_down) { //600
                    ok1=false;
                    ok2=false;
                    ok3=false;
                    ok4=false;
                    ok5=true;
                    ok6=false;
                    //while(arm.getCurrentPosition() != 160)
                    //arm.setTargetPosition(160);
                    //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //arm.setPower(1.0);
                    arm.setTargetPosition(600);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.75);
                    arm2.setPower(-0.75);
                    while (arm.isBusy()&&ok5==true);
                    arm.setPower(0);
                    arm2.setPower(0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    ok1=true;
                    ok2=true;
                    ok3=true;
                    ok4=true;
                    ok6=true;
                }

                if (gamepad2.dpad_up) {//30
                    ok1=false;
                    ok2=false;
                    ok3=false;
                    ok4=false;
                    ok5=false;
                    ok6=true;
                    //while(arm.getCurrentPosition() != 160)
                    //arm.setTargetPosition(160);
                    //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //arm.setPower(1.0);
                    arm.setTargetPosition(70);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.75);
                    arm2.setPower(0.75);
                    while (arm.isBusy()&&ok6==true);
                    arm.setPower(0);
                    arm2.setPower(0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    ok1=true;
                    ok2=true;
                    ok3=true;
                    ok4=true;
                    ok5=true;
                }

                if (gamepad2.x) {
                    ok1=false;
                    ok2=true;
                    ok3=false;
                    ok4=false;
                    ok5=false;
                    ok6=false;
                    //while(arm.getCurrentPosition() != 160)
                    //arm.setTargetPosition(160);
                    //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //arm.setPower(1.0);
                    arm.setTargetPosition(0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1.0);
                    arm2.setPower(1.0);
                    while (arm.isBusy()&&ok2==true) ;
                    arm.setPower(0);
                    arm2.setPower(0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastTime = System.currentTimeMillis();
                    ok1=true;
                    ok3=true;
                    ok4=true;
                    ok5=true;
                    ok6=true;
                }


                /*
                if(gamepad2.right_bumper){
                    arm.setTargetPosition(180);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(-1);
                    arm.setTargetPosition(400);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(-1);
                }
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                 */
            }
        }
    });
    public void stop(){stop = true;}
    public void loop(){
        telemetry.addData("DistGheara", DistGheara.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Brat pozitie: ", arm.getCurrentPosition());
        telemetry.addData("gheara stanga: ", claw_left.getPosition());
        telemetry.addData("gheara dreapta: ", claw_right.getPosition());
        //telemetry.addData("loader: ", loader.getPosition());
        //telemetry.addData("cupa: ", cupa.getPosition());
        telemetry.addData("Poz: ", poz);
        telemetry.addData("inchis: ", inchis);
        telemetry.addData("inchis2: ", inchis2);
        telemetry.addData("permisie: ", permisie);
        telemetry.addData("asdf: ", gamepad1.right_stick_y);
        telemetry.addData("thread: ", tru);
        telemetry.addData("ok1: ", ok1);
        telemetry.addData("ok2: ", ok2);
        telemetry.addData("ok3: ", ok3);
        telemetry.addData("ok4: ", ok4);
        telemetry.addData("ok5: ", ok5);
        telemetry.addData("ok6: ", ok6);
        telemetry.update();
    }
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}

