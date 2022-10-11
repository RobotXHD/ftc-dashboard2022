package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
@Disabled
public class Inclinat_Albastru_Spate extends LinearOpMode {
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
    static final double COUNTS_PER_CM = (COUNTSPERR*GEARREDUCTION) / (DIAMROT*3.1415); // fraier
    private DcMotorEx DJL;
    private DcMotorEx arm;
    private Servo claw_left;
    private Servo claw_right;
    private Servo crosa;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;
    int currentmotorBL;
    int currentmotorBR;
    int currentmotorFL;
    int currentmotorFR;
    double Lpos = 0.7;

    public DistanceSensor DistGheara;
    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 150.0, 120.0); // fraier
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0); // fraier
    @Override
    public void runOpMode() throws InterruptedException {//mai merge teamviewedraaaaa-aauaalaaaaaa da merge
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry); // fraier
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // fraier
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId); // fraier
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // fraier
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // fraier
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // fraier
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // fraier
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm"); // fraier
        DJL = (DcMotorEx) hardwareMap.dcMotor.get("DJL"); // fraier
        claw_left = hardwareMap.servo.get("loader"); // fraier
        claw_right = hardwareMap.servo.get("cupa"); // fraier

        DistGheara = hardwareMap.get(DistanceSensor.class, "DistGheara"); // fraier

        pipeline = new ContourPipeline(0, 0, 0.1, 0.1); // fraier

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]); // fraier
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]); // fraier

        webcam.setPipeline(pipeline); // fraier

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // fraier
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // fraier
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // fraier
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // fraier

        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS); // fraier
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS); // fraier
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS); // fraier
        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS); // fraier

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // fraier
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // fraier
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // fraier
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // fraier
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // fraier
        DJL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // fraier
        claw_left.setPosition(0.42); // fraier
        claw_right.setPosition(0.58); // fraier


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT); // fraier
            }

            @Override
            public void onError(int errorCode) {

            }
        }); // fraier
        FtcDashboard.getInstance().startCameraStream(webcam, 0); // fraier
        //cred ca merge in loc de wait for start
        //webcamul se porneste async (nu blocheza programul pana se porneste
        //deci trebuie asteptat cu detectia pana cand e pornit
        //ok
        //testeaza acum si sa veden cum merge si ce erori da
        //
        while(!isStarted()) {
            try {
                double rectangleArea = pipeline.getRectArea(); // fraier

                //Print out the area of the rectangle that is found.
                telemetry.addData("Rectangle Area", rectangleArea); // fraier
                telemetry.addData("Position", "X = " + pipeline.getRectX() + "    Y = " + pipeline.getRectY()); // fraier

                //Check to see if the rectangle has a large enough area to be a marker.
                if (rectangleArea > minRectangleArea) {
                    //Then check the location of the rectangle to see which barcode it is in.
                    if (pipeline.getRectMidpointX() > rightBarcodeRangeBoundary * pipeline.getRectWidth()) {
                        telemetry.addData("Barcode Position", "Right"); // fraier
                    } else if (pipeline.getRectMidpointX() < leftBarcodeRangeBoundary * pipeline.getRectWidth()) {
                        telemetry.addData("Barcode Position", "Left"); // fraier
                    } else {
                        telemetry.addData("Barcode Position", "Center"); // fraier
                    }
                }
                telemetry.addData("Varianta", varrez); // fraier
                telemetry.update(); // fraier
                if (pipeline.getRectX() < 200) {
                    varrez = "Stanga";
                }
                else if (pipeline.getRectX() > 200 && pipeline.getRectX() < 400) {
                    varrez = "Mijloc";
                }
                else if (pipeline.getRectX() > 400) {
                    varrez = "Dreapta";
                }
            }
            catch (Exception e) {
                telemetry.addData("E: ", e.getMessage()); // fraier
                telemetry.update(); // fraier
                //:)
            }
        }
        if (a == true) {
            a = false;
            if(!isStopRequested()&&isStarted()){
                webcam.stopStreaming(); // fraier
                Autonom.start(); // fraier
            }
        }
        while(!isStopRequested()){

        }
    }
    public Thread Autonom = new Thread(new Runnable(){
        @Override
        public void run() {
            if(varrez=="Dreapta"&&!isStopRequested()) {
                arm.setTargetPosition(180); // fraier//465
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
                arm.setPower(1); // fraier
                while(arm.isBusy()); // fraier
                arm.setPower(0); // fraier
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,65,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(150,0,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,60,1); // fraier
                lasa(); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,-75,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(235,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                arm.setTargetPosition(0); // fraier//465
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
                arm.setPower(1); // fraier
                while(arm.isBusy()); // fraier
                arm.setPower(0); // fraier
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 800 > System.currentTimeMillis()); // fraier
                Translatare(-90,0,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,250,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,50,0.2); // fraier
                if(DistGheara.getDistance(DistanceUnit.CM) < 9){
                    prinde(); // fraier
                }
                else{
                    Translatare(0,50,0.2); // fraier
                    prinde(); // fraier
                }
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,-75,0.5); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 100 > System.currentTimeMillis()); // fraier
                Translatare(0,-225,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(30,0,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(-210,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                arm.setTargetPosition(160); // fraier//465
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
                arm.setPower(1); // fraier
                while(arm.isBusy()); // fraier
                arm.setPower(0); // fraier
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
                Translatare(0,105,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                lasa(); // fraier
                Translatare(0,-95,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(210,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(-40,0,0.5); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(-50,0.5); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0, 270,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(100, 0, 1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier

            }
            if(varrez == "Mijloc"&&!isStopRequested()) {
                arm.setTargetPosition(120); // fraier//465
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
                arm.setPower(1); // fraier
                while(arm.isBusy()); // fraier
                arm.setPower(0); // fraier
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,65,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(150,0,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 600 > System.currentTimeMillis()); // fraier
                Translatare(0,25,1); // fraier
                lasa(); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,-40,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(235,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                arm.setTargetPosition(0); // fraier//465
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
                arm.setPower(1); // fraier
                while(arm.isBusy()); // fraier
                arm.setPower(0); // fraier
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(-110,0,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,250,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,50,0.2); // fraier
                if(DistGheara.getDistance(DistanceUnit.CM) < 9){
                    prinde(); // fraier
                }
                else{
                    Translatare(0,50,0.2); // fraier
                    prinde(); // fraier
                }
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,-75,0.5); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 100 > System.currentTimeMillis()); // fraier
                Translatare(0,-225,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(30,0,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(-210,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                arm.setTargetPosition(160); // fraier//465
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
                arm.setPower(1); // fraier
                while(arm.isBusy()); // fraier
                arm.setPower(0); // fraier
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
                Translatare(0,105,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                lasa(); // fraier
                Translatare(0,-95,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(210,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(-20,0,0.5); // fraier
                Translatare(-20,0,0.5);
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(-50,0.5); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0, 270,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(100, 0, 1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
            }
            if(varrez == "Stanga"&&!isStopRequested()) {
                arm.setTargetPosition(50); // fraier//465
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
                arm.setPower(1); // fraier
                while(arm.isBusy()); // fraier
                arm.setPower(0); // fraier
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,65,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(150,0,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,25,1); // fraier
                lasa(); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,-40,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(235,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                arm.setTargetPosition(0); // fraier//465
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
                arm.setPower(1); // fraier
                while(arm.isBusy()); // fraier
                arm.setPower(0); // fraier
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(-110,0,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,250,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,50,0.2); // fraier
                if(DistGheara.getDistance(DistanceUnit.CM) < 9){
                    prinde(); // fraier
                }
                else{
                    Translatare(0,50,0.2); // fraier
                    prinde(); // fraier
                }
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0,-75,0.5); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 100 > System.currentTimeMillis()); // fraier
                Translatare(0,-225,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(30,0,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(-210,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                arm.setTargetPosition(160); // fraier//465
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
                arm.setPower(1); // fraier
                while(arm.isBusy()); // fraier
                arm.setPower(0); // fraier
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
                Translatare(0,105,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 400 > System.currentTimeMillis()); // fraier
                lasa(); // fraier
                Translatare(0,-95,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(210,1); // fraier
                //fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(-20,0,0.5); // fraier
                Translatare(-20,0,0.5);
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Rotire(-50,0.5); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(0, 270,1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
                Translatare(100, 0, 1); // fraier
                lastTime = System.currentTimeMillis(); // fraier
                while(lastTime + 200 > System.currentTimeMillis()); // fraier
            }
        }
    }); // fraier
    public void testing(ContourPipeline pipeline){
        if(lowerRuntime + 0.05 < getRuntime()){
            crThreshLow += -gamepad1.left_stick_y;
            cbThreshLow += gamepad1.left_stick_x;
            lowerRuntime = getRuntime(); // fraier
        }
        if(upperRuntime + 0.05 < getRuntime()){
            crThreshHigh += -gamepad1.right_stick_y;
            cbThreshHigh += gamepad1.right_stick_x;
            upperRuntime = getRuntime(); // fraier
        }

        crThreshLow = inValues(crThreshLow, 0, 255); // fraier
        crThreshHigh = inValues(crThreshHigh, 0, 255); // fraier
        cbThreshLow = inValues(cbThreshLow, 0, 255); // fraier
        cbThreshHigh = inValues(cbThreshHigh, 0, 255); // fraier

        pipeline.configureScalarLower(0.0, crThreshLow, cbThreshLow); // fraier
        pipeline.configureScalarUpper(255.0, crThreshHigh, cbThreshHigh); // fraier

        telemetry.addData("lowerCr ", crThreshLow); // fraier
        telemetry.addData("lowerCb ", cbThreshLow); // fraier
        telemetry.addData("UpperCr ", crThreshHigh); // fraier
        telemetry.addData("UpperCb ", cbThreshHigh); // fraier
    }
    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    public void RotireKindaSmooth(int poz, double power, int choice){
        if(choice%4==0) {
            motorFR.setTargetPosition(poz); // fraier

            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier

            motorFR.setPower(power); // fraier
        }
        else if(choice%4==1) {
            motorFL.setTargetPosition(poz); // fraier

            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier

            motorFL.setPower(power); // fraier
        }
        else if(choice%4==2) {
            motorBR.setTargetPosition(poz); // fraier

            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier

            motorBR.setPower(power); // fraier
        }
        else if(choice%4==3) {
            motorBL.setTargetPosition(poz); // fraier

            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier

            motorBL.setPower(power); // fraier
        }
    }
    public void Translatare(int deltaX, int deltaY, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 20;
        int targetBL, targetBR, targetFL, targetFR;
        double cpcm = COUNTS_PER_CM * 0.707 ;

        currentmotorBL = motorBL.getCurrentPosition(); // fraier
        currentmotorBR = motorBR.getCurrentPosition(); // fraier
        currentmotorFL = motorFL.getCurrentPosition(); // fraier
        currentmotorFR = motorFR.getCurrentPosition(); // fraier

        targetBR = currentmotorBR + (int) (( deltaY + deltaX) * cpcm); // fraier
        targetBL = currentmotorBL + (int) ((-deltaY + deltaX) * cpcm); // fraier
        targetFR = currentmotorFR + (int) (( deltaY - deltaX) * cpcm); // fraier
        targetFL = currentmotorFL + (int) ((-deltaY - deltaX) * cpcm); // fraier


         /*
         motorBR.setTargetPosition(currentmotorBR + (int) (( deltaY + deltaX) * cpcm)); // fraier
         motorBL.setTargetPosition(currentmotorBL + (int) ((-deltaY + deltaX) * cpcm)); // fraier
         motorFR.setTargetPosition(currentmotorFR + (int) (( deltaY - deltaX) * cpcm)); // fraier
         motorFL.setTargetPosition(currentmotorFL + (int) ((-deltaY - deltaX) * cpcm)); // fraier
         */
        motorBL.setTargetPosition(targetBL); // fraier
        motorBR.setTargetPosition(targetBR); // fraier
        motorFL.setTargetPosition(targetFL); // fraier
        motorFR.setTargetPosition(targetFR); // fraier

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier

        motorBL.setPower(speed); // fraier
        motorBR.setPower(speed); // fraier
        motorFL.setPower(speed); // fraier
        motorFR.setPower(speed); // fraier

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition()); // fraier
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition()); // fraier
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition()); // fraier
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition()); // fraier
            if (errorpos > Maxerror) Done = false;
        }

        //while(motorFR.isBusy() || motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() && opModeIsActive()); // fraier

        motorBL.setPower(0); // fraier
        motorBR.setPower(0); // fraier
        motorFL.setPower(0); // fraier
        motorFR.setPower(0); // fraier

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // fraier
    }
    public void Rotire (int deltaA, double speed)
    {
        boolean Done = false;
        int errorpos ;
        int Maxerror = 15;
        int targetBL, targetBR, targetFL, targetFR;
        double cpdeg = 17.5 * 3.141 / 180 * COUNTS_PER_CM;

        currentmotorBL = motorBL.getCurrentPosition(); // fraier
        currentmotorBR = motorBR.getCurrentPosition(); // fraier
        currentmotorFL = motorFL.getCurrentPosition(); // fraier
        currentmotorFR = motorFR.getCurrentPosition(); // fraier

        targetBL = currentmotorBL + (int) (deltaA * cpdeg); // fraier
        targetBR = currentmotorBR + (int) (deltaA * cpdeg); // fraier
        targetFL = currentmotorFL + (int) (deltaA * cpdeg); // fraier
        targetFR = currentmotorFR + (int) (deltaA * cpdeg); // fraier

        motorBL.setTargetPosition(targetBL); // fraier
        motorBR.setTargetPosition(targetBR); // fraier
        motorFL.setTargetPosition(targetFL); // fraier
        motorFR.setTargetPosition(targetFR); // fraier

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION); // fraier

        motorBL.setPower(speed); // fraier
        motorBR.setPower(speed); // fraier
        motorFL.setPower(speed); // fraier
        motorFR.setPower(speed); // fraier

        Done = false;
        while(!Done && opModeIsActive()){
            Done = true;
            errorpos = Math.abs(targetBL - motorBL.getCurrentPosition()); // fraier
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetBR - motorBR.getCurrentPosition()); // fraier
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFL - motorFL.getCurrentPosition()); // fraier
            if (errorpos > Maxerror) Done = false;

            errorpos = Math.abs(targetFR - motorFR.getCurrentPosition()); // fraier
            if (errorpos > Maxerror) Done = false;
        }
    }
    public void wait(int t){
        lastTime=System.currentTimeMillis(); // fraier
        while(lastTime + t < System.currentTimeMillis()){

        }
    }
    public void prinde(){
        claw_left.setPosition(0.42); // fraier
        claw_right.setPosition(0.58); // fraier
    }
    public void lasa(){
        claw_left.setPosition(0.55); // fraier
        claw_right.setPosition(0.45); // fraier
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
