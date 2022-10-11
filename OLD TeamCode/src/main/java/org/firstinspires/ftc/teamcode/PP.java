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
public class PP extends OpMode{
    //private Gyroscope imu;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFL;
    private DcMotor motorFR;
    //private DcMotor arm,arm2;
    //private Servo claw;
    double sm = 1, ms = 1, ok0 = 0;
    double poz = 0;
    double gpoz = 0.5;
    double y, x, rx;
    //double armPower, slidePower;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    //public DistanceSensor DistGheara;
    double lastTime;
    float right_stick2;
    float right_stick1;
    boolean v = true, ok1,ok2,ok3,ok4,ok5,ok6,ok7;
    boolean FirstTime = true;
    boolean inchis = false;
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
        /*arm = hardwareMap.get(DcMotor.class, "arm");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        claw = hardwareMap.servo.get("claw");*/
       // miscator = hardwareMap.servo.get("miscator");

        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //arm.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //arm2.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Resseting", "Encoders");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        // run until the end of the match (driver presses STOP)

    }
    @Override
    public void start(){
        Chassis.start();
        Systems.start();
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
                    ok7 = false;
                }
                if(gamepad2.right_bumper){
                    ok1 = true;
                    ok2 = true;
                    ok3 = true;
                    ok4 = true;
                    ok5 = true;
                    ok6 = true;
                    ok7 = true;
                }
                if(gamepad1.left_trigger!=0) {
                    ok1 = false;
                    ok2 = false;
                    ok3 = false;
                    ok4 = false;
                    ok5 = false;
                    ok6 = false;
                    ok7 = false;
                }
                if(gamepad1.right_trigger!=0){
                    ok1 = true;
                    ok2 = true;
                    ok3 = true;
                    ok4 = true;
                    ok5 = true;
                    ok6 = true;
                    ok7 = true;
                }
                tru = true;

                y  = -gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x * 1.5;
                rx = gamepad1.right_stick_x;

                pmotorFL = -y - x - rx;
                pmotorBL = -y + x - rx;
                pmotorBR = -y - x + rx;
                pmotorFR = -y + x + rx;

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
                    }
                    else {
                        sm = 0.5;
                        POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
                    }
                }
            }
        }
    });
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {

               // armPower  = gamepad2.left_stick_y;
                //arm.setPower(armPower / ms);
                //arm2.setPower(armPower / ms);
                if(gamepad2.right_bumper)
                    ms = 2;
                else if (gamepad2.left_bumper)
                    ms = 5;
                else ms = 0.5;

                //claw.setPosition(gamepad2.right_stick_x);
                /*if (gamepad2.b && ok0==0){
                    ok0=1;
                    miscator.setPosition(0.1);
                }
                else if (gamepad2.b && ok0==1) {
                    ok0=2;
                    miscator.setPosition(0.2);
                }
                else if (gamepad2.b && ok0==2){
                    ok0=0;
                    miscator.setPosition(0.3);
                }*/
            }
        }
    });
    public void stop(){stop = true;}
    public void loop(){
        //telemetry.addData("Left Bumper", armPower);
        telemetry.addData("Poz: ", gamepad2.left_stick_y);
        telemetry.addData("permisie: ", permisie);
        telemetry.addData("asdf: ", gamepad1.right_stick_y);
        telemetry.addData("thread: ", tru);
        telemetry.addData("ok0: ", gamepad2.a);
        telemetry.addData("ok1: ", gamepad2.y);
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

