package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class pidul_meu extends OpMode {
    public DcMotorEx motorFR,motorFL,motorBR,motorBL;
    double y, x, rx, dBL,dBR,dFL,dFR;
    double max = 0;
    double pmotorBL,pmotor2BL=0;
    double pmotorBR,pmotor2BR=0;
    double pmotorFL,pmotor2FL=0;
    double pmotorFR,pmotor2FR=0;
    double sm = 1;
    boolean bum = true;
    private boolean stop;
    private ColorSensor colorR,colorL;
    private DistanceSensor distR, distL, distF, distB;
    @Override
    public void init() {
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Front-Right

        distL = hardwareMap.get(DistanceSensor.class, "distL");
        distR = hardwareMap.get(DistanceSensor.class, "distR");
        distB = hardwareMap.get(DistanceSensor.class, "distB");
        distF = hardwareMap.get(DistanceSensor.class, "distF");

        colorL = hardwareMap.get(ColorSensor.class, "colorL");
        colorR = hardwareMap.get(ColorSensor.class, "colorR");

        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stop(){stop = true;}
    @Override
    public void start(){
        Lamisto.start();
        Sensors.start();
    }
    private final Thread Lamisto = new Thread(new Runnable(){
        @Override
        public void run() {
            while(!stop) {
                if (gamepad1.left_bumper && bum == false) {
                    bum = true;
                } else if (gamepad1.left_bumper && bum == true) {
                    bum = false;
                }
                if(distR.getDistance(DistanceUnit.CM) < 10){
                    x = 1;
                }
                else if(distL.getDistance(DistanceUnit.CM) < 10){
                    x = -1;
                }
                else {
                    x = gamepad1.left_stick_x;
                }
                if(distF.getDistance(DistanceUnit.CM) < 10){
                    y = -1;
                }
                else if(distB.getDistance(DistanceUnit.CM) < 10){
                    y = 1;
                }
                else {
                    y = gamepad1.left_stick_y;
                }
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
                if (bum == false) {
                    sm = 1;
                }
                else {
                    sm = 2;
                }
                POWER(pmotorFR / sm + pmotor2FR / sm, pmotorFL / sm + pmotor2FL / sm , pmotorBR / sm + pmotor2BR / sm, pmotorBL / sm + pmotor2BL / sm);
            }
        }
    });
    private final Thread Sensors = new Thread(new Runnable(){
        @Override
        public void run() {
            while(!stop){
                dFR = distF.getDistance(DistanceUnit.CM);
                dFL = distB.getDistance(DistanceUnit.CM);
                dFR = distR.getDistance(DistanceUnit.CM);
                dFR = distL.getDistance(DistanceUnit.CM);
            }
        }
    });
    @Override
    public void loop() {
        telemetry.addData("puteremotorFR:", pmotorFR);
        telemetry.addData("puteremotorFL:", pmotorFL);
        telemetry.addData("puteremotorBR:", pmotorBR);
        telemetry.addData("puteremotorBL:", pmotorBL);
        telemetry.addData("distanta front-right:", distF.getDistance(DistanceUnit.CM));
        telemetry.addData("distanta front-left:", distB.getDistance(DistanceUnit.CM));
        telemetry.addData("distanta bottom-right:", distR.getDistance(DistanceUnit.CM));
        telemetry.addData("distanta bottom-left:", distL.getDistance(DistanceUnit.CM));
        telemetry.addData("RedL  ", colorL.red());
        telemetry.addData("GreenL", colorL.green());
        telemetry.addData("BlueL ", colorL.blue());
        telemetry.addData("RedR  ", colorR.red());
        telemetry.addData("GreenR", colorR.green());
        telemetry.addData("BlueR ", colorR.blue());
        telemetry.addData("AlphaR",colorR.alpha());
        telemetry.addData("AlphaL",colorL.alpha());
    }
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}







































































































































































































































































































































































































































































































































































































