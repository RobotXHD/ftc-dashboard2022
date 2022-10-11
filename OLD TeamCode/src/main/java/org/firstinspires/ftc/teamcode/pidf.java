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
//hello
//care e problema?
//stai ca trebuie sa aflu cum se instaleaza gradle
//ca de aia nu merge
//stai un pic
//pai nu putem uploada codul si is toaate rosii ;(
//stai ca deocamdata nu putem face un nou java class ca nu avem chestia necesara pentru aia
@TeleOp
public class pidf extends OpMode {
    //private Gyroscope imu;
    private DcMotorEx motorBL;
    private DcMotorEx motorBR;
    private DcMotorEx motorFL;
    private DcMotorEx motorFR;
    double sm = 1, ms = 1, ok0 = 0;
    double poz = 0;
    double gpoz = 0.5;
    double y, x, rx;
    double max = 0;
    double currentVelocityFL;
    double currentVelocityBR;
    double currentVelocityFR;
    double currentVelocityBL;
    double maxVelocityFL;
    double maxVelocityBR;
    double maxVelocityFR;
    double maxVelocityBL;
    double lastTime;
    float right_stick2;
    float right_stick1;
    boolean v = true, ok1, ok2, ok3, ok4, ok5, ok6, ok7;
    boolean FirstTime = true;
    boolean inchis = false;
    boolean overpower = true;
    boolean permisie = true;
    boolean stopDJ = false;
    boolean tru = false;
    private boolean stop;
    int okGrip = 1;
    public ElapsedTime timer = new ElapsedTime();
    double timeLimit = 0.25;
    int loaderState = -1;


    public void init() {
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Front-Right

        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Resseting", "Encoders");
        telemetry.update();

    }

    @Override
    public void start() {
        Chassis.start();
        Systems.start();
    }

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if (gamepad2.left_bumper) {
                    ok1 = false;
                    ok2 = false;
                    ok3 = false;
                    ok4 = false;
                    ok5 = false;
                    ok6 = false;
                    ok7 = false;
                }
                if (gamepad2.right_bumper) {
                    ok1 = true;
                    ok2 = true;
                    ok3 = true;
                    ok4 = true;
                    ok5 = true;
                    ok6 = true;
                    ok7 = true;
                }
                if (gamepad1.left_trigger != 0) {
                    ok1 = false;
                    ok2 = false;
                    ok3 = false;
                    ok4 = false;
                    ok5 = false;
                    ok6 = false;
                    ok7 = false;
                }
                if (gamepad1.right_trigger != 0) {
                    ok1 = true;
                    ok2 = true;
                    ok3 = true;
                    ok4 = true;
                    ok5 = true;
                    ok6 = true;
                    ok7 = true;
                }
                tru = true;

                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x * 1.5;
                rx = gamepad1.right_stick_x;

                currentVelocityFL = -y - x - rx;
                currentVelocityBL = -y + x - rx;
                currentVelocityBR = -y - x + rx;
                currentVelocityFR = -y + x + rx;

                max = abs(currentVelocityFL);
                if (abs(currentVelocityFR) > max) {
                    max = abs(currentVelocityFR);
                }
                if (abs(currentVelocityBL) > max) {
                    max = abs(currentVelocityBL);
                }
                if (abs(currentVelocityBR) > max) {
                    max = abs(currentVelocityBR);
                }
                if (max > 1) {
                    currentVelocityFL /= max;
                    currentVelocityFR /= max;
                    currentVelocityBL /= max;
                    currentVelocityBR /= max;
                }

                currentVelocityFL = motorFL.getVelocity();
                if (currentVelocityFL > maxVelocityFL) {
                    maxVelocityFL = currentVelocityFL;
                }

                currentVelocityFR = motorFR.getVelocity();
                if (currentVelocityFR > maxVelocityFR) {
                    maxVelocityFR = currentVelocityFR;
                }

                currentVelocityBR = motorBR.getVelocity();
                if (currentVelocityBR > maxVelocityBR) {
                    maxVelocityBR = currentVelocityBR;
                }

                currentVelocityBL = motorBL.getVelocity();
                if (currentVelocityBL > maxVelocityBL) {
                    maxVelocityBL = currentVelocityBL;
                }
            }
        }
    });
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {

                if (gamepad2.right_bumper)
                    ms = 2;
                else if (gamepad2.left_bumper)
                    ms = 5;
                else ms = 0.5;
            }
        }
    });

    public void stop() {
        stop = true;
    }

    public void loop() {
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

        telemetry.addData("current velocity FL", currentVelocityFL);
        telemetry.addData("maximum velocity FL", maxVelocityFL);

        telemetry.addData("current velocity FR", currentVelocityFR);
        telemetry.addData("maximum velocity FR", maxVelocityFR);

        telemetry.addData("current velocity BL", currentVelocityBL);
        telemetry.addData("maximum velocity BL", maxVelocityBL);

        telemetry.addData("current velocity BR", currentVelocityBR);
        telemetry.addData("maximum velocity BR", maxVelocityBR);
        telemetry.update();
        telemetry.update();
    }

    public void POWER(double df1, double sf1, double ds1, double ss1) {
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}

