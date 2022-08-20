package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class Schimbare extends LinearOpMode {
    private OpenCvCamera webcam;
    private ContourPipeline pipeline;
    DcMotorEx motorFR, motorFL, motorBR, motorBL;
    int ok=0,pduck=0;
    public double lastTime;
    boolean a=true;

    private static final String cub = "Cube";
    String varrez = "Dreapta";
    static final double COUNTSPERR = 383.6;
    static final double GEARREDUCTION = 1;
    static final double DIAMROT = 9.6;
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415);
    private DcMotorEx DJL;
    private DcMotorEx arm;
    private Servo claw_left;
    private Servo claw_right;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;
    double Lpos = 0.7;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 150.0, 120.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);
    @Override
    public void runOpMode() throws InterruptedException {//mai merge teamviewedraaaaa-aauaalaaaaaa da merge
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        DJL = (DcMotorEx) hardwareMap.dcMotor.get("DJL");
        claw_left = hardwareMap.servo.get("loader");
        claw_right = hardwareMap.servo.get("cupa");

        pipeline = new ContourPipeline(0, 0, 0.1, 0.1);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DJL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw_left.setPosition(0.42);
        claw_right.setPosition(0.58);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        //cred ca merge in loc de wait for start
        //webcamul se porneste async (nu blocheza programul pana se porneste
        //deci trebuie asteptat cu detectia pana cand e pornit
        //ok
        //testeaza acum si sa veden cum merge si ce erori da
        //
        while(!isStarted()) {
            try {
                double rectangleArea = pipeline.getRectArea();

                //Print out the area of the rectangle that is found.
                telemetry.addData("Rectangle Area", rectangleArea);
                telemetry.addData("Position", "X = " + pipeline.getRectX() + "    Y = " + pipeline.getRectY());

                //Check to see if the rectangle has a large enough area to be a marker.
                if (rectangleArea > minRectangleArea) {
                    //Then check the location of the rectangle to see which barcode it is in.
                    if (pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * pipeline.getRectWidth()) {
                        telemetry.addData("Barcode Position", "Right");
                    } else if (pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * pipeline.getRectWidth()) {
                        telemetry.addData("Barcode Position", "Left");
                    } else {
                        telemetry.addData("Barcode Position", "Center");
                    }
                }
                telemetry.addData("Varianta", varrez);
                telemetry.update();
                if (pipeline.getRectX() < 250) {
                    varrez = "Stanga";
                }
                else if (pipeline.getRectX() > 250 && pipeline.getRectX() < 450) {
                    varrez = "Mijloc";
                }
                else if (pipeline.getRectX() > 450) {
                    varrez = "Dreapta";
                }
            }
            catch (Exception e) {
                telemetry.addData("E: ", e.getMessage());
                telemetry.update();
                //:)
            }
        }
        if (a == true) {
            a = false;
            if(!isStopRequested()&&isStarted()){
                webcam.stopStreaming();
                Autonom.start();
            }
        }

        while(!isStopRequested()){

        }
    }
    public Thread Autonom = new Thread(new Runnable(){
        @Override
        public void run() {
            if(varrez=="Dreapta"&&!isStopRequested()) {

                arm.setTargetPosition(150);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while(arm.isBusy());
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastTime = System.currentTimeMillis();
                while(lastTime + 100 > System.currentTimeMillis()){
                }

                Translatare(-136, 30, 0.6);
                Translatare(0, -3, 0.6);
                //Translatare(-45, 30, 0.3);
                lastTime = System.currentTimeMillis();
                while(lastTime + 200 > System.currentTimeMillis()){
                }

                DJL.setPower(0.50);
                lastTime = System.currentTimeMillis();
                while(lastTime + 2300 > System.currentTimeMillis()){
                }

                DJL.setPower(0.0);

                Translatare(0, 165, 0.7);
                Rotire(-220, 0.7);
                Translatare(0, 150, 0.7);


                claw_left.setPosition(0.55);
                claw_right.setPosition(0.45);
                lastTime = System.currentTimeMillis();
                while(lastTime + 500 > System.currentTimeMillis()){
                }

                //Rotire(220, 0.9);

                Translatare(0, -135, 0.75);
                Translatare(70, 0, 0.75);

                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while(arm.isBusy());
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }

            if(varrez == "Mijloc"&&!isStopRequested())
            {

                arm.setTargetPosition(92);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while(arm.isBusy());
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastTime = System.currentTimeMillis();
                while(lastTime + 100 > System.currentTimeMillis()){
                }

                Translatare(-136, 30, 0.6);
                Translatare(0, -3, 0.6);
                //Translatare(-45, 30, 0.3);
                lastTime = System.currentTimeMillis();
                while(lastTime + 200 > System.currentTimeMillis()){
                }

                DJL.setPower(0.50);
                lastTime = System.currentTimeMillis();
                while(lastTime + 2300 > System.currentTimeMillis()){
                }

                DJL.setPower(0.0);

                Translatare(0, 165, 0.7);
                Rotire(-220, 0.7);
                Translatare(0, 150, 0.7);


                claw_left.setPosition(0.55);
                claw_right.setPosition(0.45);
                lastTime = System.currentTimeMillis();
                while(lastTime + 500 > System.currentTimeMillis()){
                }

                //Rotire(220, 0.9);

                Translatare(0, -135, 0.75);
                Translatare(70, 0, 0.75);

                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while(arm.isBusy());
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            }

            if(varrez == "Stanga"&&!isStopRequested())
            {
                arm.setTargetPosition(45);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while(arm.isBusy());
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastTime = System.currentTimeMillis();
                while(lastTime + 100 > System.currentTimeMillis()){
                }

                Translatare(-136, 30, 0.6);
                Translatare(0, -3, 0.6);
                //Translatare(-45, 30, 0.3);
                lastTime = System.currentTimeMillis();
                while(lastTime + 200 > System.currentTimeMillis()){
                }

                DJL.setPower(0.50);
                lastTime = System.currentTimeMillis();
                while(lastTime + 2300 > System.currentTimeMillis()){
                }

                DJL.setPower(0.0);

                Translatare(0, 165, 0.7);
                Rotire(-220, 0.7);
                Translatare(0, 153, 0.7);


                claw_left.setPosition(0.55);
                claw_right.setPosition(0.45);
                lastTime = System.currentTimeMillis();
                while(lastTime + 500 > System.currentTimeMillis()){
                }

                //Rotire(220, 0.9);

                Translatare(0, -135, 0.75);
                Translatare(70, 0, 0.75);

                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                while(arm.isBusy());
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }
        }
    });
    public void testing(ContourPipeline pipeline){
        if(lowerRuntime + 0.05 < getRuntime()){
            crThreshLow += -gamepad1.left_stick_y;
            cbThreshLow += gamepad1.left_stick_x;
            lowerRuntime = getRuntime();
        }
        if(upperRuntime + 0.05 < getRuntime()){
            crThreshHigh += -gamepad1.right_stick_y;
            cbThreshHigh += gamepad1.right_stick_x;
            upperRuntime = getRuntime();
        }

        crThreshLow = inValues(crThreshLow, 0, 255);
        crThreshHigh = inValues(crThreshHigh, 0, 255);
        cbThreshLow = inValues(cbThreshLow, 0, 255);
        cbThreshHigh = inValues(cbThreshHigh, 0, 255);

        pipeline.configureScalarLower(0.0, crThreshLow, cbThreshLow);
        pipeline.configureScalarUpper(255.0, crThreshHigh, cbThreshHigh);

        telemetry.addData("lowerCr ", crThreshLow);
        telemetry.addData("lowerCb ", cbThreshLow);
        telemetry.addData("UpperCr ", crThreshHigh);
        telemetry.addData("UpperCb ", cbThreshHigh);
    }
    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    public void RotireKindaSmooth(int poz, double power, int choice){
        if(choice%4==0) {
            motorFR.setTargetPosition(poz);

            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFR.setPower(power);
        }
        else if(choice%4==1) {
            motorFL.setTargetPosition(poz);

            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorFL.setPower(power);
        }
        else if(choice%4==2) {
            motorBR.setTargetPosition(poz);

            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBR.setPower(power);
        }
        else if(choice%4==3) {
            motorBL.setTargetPosition(poz);

            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorBL.setPower(power);
        }
    }
    public void Translatare(int deltaX, int deltaY, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 20;
        int targetBL, targetBR, targetFL, targetFR;
        double cpcm = COUNTS_PER_CM * 0.707 ;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBR = currentmotorBR + (int) (( deltaY + deltaX) * cpcm);
        targetBL = currentmotorBL + (int) ((-deltaY + deltaX) * cpcm);
        targetFR = currentmotorFR + (int) (( deltaY - deltaX) * cpcm);
        targetFL = currentmotorFL + (int) ((-deltaY - deltaX) * cpcm);


         /*
         motorBR.setTargetPosition(currentmotorBR + (int) (( deltaY + deltaX) * cpcm));
         motorBL.setTargetPosition(currentmotorBL + (int) ((-deltaY + deltaX) * cpcm));
         motorFR.setTargetPosition(currentmotorFR + (int) (( deltaY - deltaX) * cpcm));
         motorFL.setTargetPosition(currentmotorFL + (int) ((-deltaY - deltaX) * cpcm));
         */
        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }

        //while(motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() && opModeIsActive());

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Rotire (int deltaA, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 15;
        int targetBL, targetBR, targetFL, targetFR;
        double cpdeg = 17.5 * 3.141 / 180 * COUNTS_PER_CM;

        currentmotorBL = motorBL.getCurrentPosition();
        currentmotorBR = motorBR.getCurrentPosition();
        currentmotorFL = motorFL.getCurrentPosition();
        currentmotorFR = motorFR.getCurrentPosition();

        targetBL = currentmotorBL + (int) (deltaA * cpdeg);
        targetBR = currentmotorBR + (int) (deltaA * cpdeg);
        targetFL = currentmotorFL + (int) (deltaA * cpdeg);
        targetFR = currentmotorFR + (int) (deltaA * cpdeg);

        motorBL.setTargetPosition(targetBL);
        motorBR.setTargetPosition(targetBR);
        motorFL.setTargetPosition(targetFL);
        motorFR.setTargetPosition(targetFR);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition());
            if (errorpos > Maxerror) Done = false;
        }
    }
    public void wait(int t){
        lastTime=System.currentTimeMillis();
        while(lastTime+t<System.currentTimeMillis()){

        }
    }
}
/*              |
                |
                |
                |
                |
________________|________________
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |
                |

 */
