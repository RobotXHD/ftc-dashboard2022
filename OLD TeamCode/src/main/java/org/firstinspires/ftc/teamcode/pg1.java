package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp
public class pg1 extends OpMode {
    public DcMotorEx motorfr;
    public DcMotorEx motorfl;
    public DcMotorEx motorsl;
    public DcMotorEx motorsr;

    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    double sm;
    @Override
    public void init() {
        motorfr = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Back-Left
        motorsl = hardwareMap.get(DcMotorEx.class, "motorSL"); // Motor Back-Right
        motorfl = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Front-Left
        motorsr = hardwareMap.get(DcMotorEx.class, "motorSR"); // Motor Front-Right
        motorsl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorfl.setDirection(DcMotorSimple.Direction.REVERSE);


        motorsl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorsr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorfr.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorfl.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorsr.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorsl.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorfr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorsl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
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
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorfr.setPower(df1);
        motorsl.setPower(ss1);
        motorfl.setPower(sf1);
        motorsr.setPower(ds1);
    }
}
/*
0.motorFR
1.motorBL
2.motorBR
3.motorFL
 */