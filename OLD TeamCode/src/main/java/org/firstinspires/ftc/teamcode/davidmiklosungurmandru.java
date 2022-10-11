package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class davidmiklosungurmandru extends LinearOpMode {
     private DcMotorEx motorFL;
     private DcMotorEx motorFR;
     private DcMotorEx motorBL;
     private DcMotorEx motorBR;
    double currentVelocityFL, currentVelocityFR, currentVelocityBL, currentVelocityBR;
    double maxVelocityFL = 0.0, maxVelocityFR = 0.0, maxVelocityBL= 0.0, maxVelocityBR = 0.0;

    @Override
    public void runOpMode() {
        motorFL= hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR= hardwareMap.get(DcMotorEx.class, "motorFR");
        waitForStart();
        while (opModeIsActive()) {

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

            telemetry.addData("current velocity FL", currentVelocityFL);
            telemetry.addData("maximum velocity FL", maxVelocityFL);

            telemetry.addData("current velocity FR", currentVelocityFR);
            telemetry.addData("maximum velocity FR", maxVelocityFR);

            telemetry.addData("current velocity BL", currentVelocityBL);
            telemetry.addData("maximum velocity BL", maxVelocityBL);

            telemetry.addData("current velocity BR", currentVelocityBR);
            telemetry.addData("maximum velocity BR", maxVelocityBR);
            telemetry.update();
        }
    }
}


