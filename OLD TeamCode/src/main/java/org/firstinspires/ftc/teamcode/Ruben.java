package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import android.media.MediaPlayer;

import java.io.IOException;
@Disabled
class CenaPlayer {
    //The player handling the audio
    private static MediaPlayer mediaPlayer = null;
    //Start the wubs
    public static void start(Context context) {
        if (mediaPlayer == null) mediaPlayer = MediaPlayer.create(context, R.raw.boom);
        mediaPlayer.seekTo(0);
        mediaPlayer.start();
    }
    //Stop the wubs
    public static void stop() {
        if (mediaPlayer != null) {
            mediaPlayer.stop();
            try { mediaPlayer.prepare(); }
            catch (IOException e) {}
        }
    }
}

@TeleOp
public class Ruben extends OpMode {
    boolean pressedLast;
    public DcMotorEx motorfr;
    public DcMotorEx motorfl;
    public DcMotorEx motorsl;
    public DcMotorEx motorsr;
    boolean cacat=true;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    double sm;
    public boolean stop = false;
    public double timpcacat;


    public Context appContext;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern,pattern2;

    @Override
    public void init() {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        pattern2 = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        blinkinLedDriver.setPattern(pattern);

        motorfr = hardwareMap.get(DcMotorEx.class, "fd"); // Motor Front-Left
        motorsl = hardwareMap.get(DcMotorEx.class, "ss"); // Motor Back-Right
        motorfl = hardwareMap.get(DcMotorEx.class, "fs"); // Motor Front-Left
        motorsr = hardwareMap.get(DcMotorEx.class, "sd"); // Motor Back-Right
        motorsl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorfl.setDirection(DcMotorSimple.Direction.REVERSE);


        motorsl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        motorfr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorfl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Girofar.start();
    }

    @Override
    public void loop() {

        if (this.gamepad1.x && !this.pressedLast) {
            CenaPlayer.start(this.hardwareMap.appContext);
            this.pressedLast = true;
        }
        else if (!this.gamepad1.x && this.pressedLast) {
            CenaPlayer.stop();
            this.pressedLast = false;
        }

        y  = -gamepad1.left_stick_y;
        x  = gamepad1.left_stick_x * 1.5;
        rx = gamepad1.right_stick_x;

        if(gamepad1.right_trigger > 0) {
            pmotorFL = gamepad1.right_trigger + rx;
            pmotorBL = gamepad1.right_trigger + rx;
            pmotorBR = gamepad1.right_trigger - rx;
            pmotorFR = gamepad1.right_trigger - rx;
        }
        else if( gamepad1.left_trigger > 0) {
            pmotorFL = -gamepad1.left_trigger + rx;
            pmotorBL = -gamepad1.left_trigger + rx;
            pmotorBR = -gamepad1.left_trigger - rx;
            pmotorFR = -gamepad1.left_trigger - rx;
        }
        else {
            pmotorFL =  rx;
            pmotorBL =  rx;
            pmotorBR =  - rx;
            pmotorFR =  - rx;
        }
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

        if(gamepad1.a)
            cacat=true;
        else if(gamepad1.y)
            cacat=false;

        //SLOW-MOTION
        if (gamepad1.left_bumper) {
            sm = 2;
            POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);

        } else {
            //SLOWER-MOTION
            if (gamepad1.right_bumper) {
                sm = 5;
                POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
            } else {
                sm = 0.1;
                POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
            }
        }
    }
    public void stop(){
        stop=true;
        CenaPlayer.stop();
    }
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorfr.setPower(df1);
        motorsl.setPower(ss1);
        motorfl.setPower(sf1);
        motorsr.setPower(ds1);
    }

    public Thread Girofar = new Thread(new Runnable() {
        @Override
        public void run() {
            while(!stop){
                if(cacat)
                {
                    timpcacat=System.currentTimeMillis();
                    blinkinLedDriver.setPattern(pattern);
                    while(timpcacat+2000==System.currentTimeMillis());
                    timpcacat=System.currentTimeMillis();
                    blinkinLedDriver.setPattern(pattern2);
                    while(timpcacat+2000==System.currentTimeMillis());
                }
                else
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
        }
    });
}
