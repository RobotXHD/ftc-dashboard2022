package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import static java.lang.Math.abs;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@TeleOp


public class VuTeleOp extends LinearOpMode {
    private static final String VUFORIA_KEY =
            "ATYhHtX/////AAABmYiQro9x40+lpw67He6N9ct0CV8iFEOYHpbJ8b4xWTZwlfuNQu2h1+PQ03af1oOQ+Wf5jInqtlVVUYnO4fT45P++kQk+pqOW6dYQu+X7hM0zl3kDZ1v1iwIMDm4t0JHmG7CRgSKRjHOx5QNdkuLN692ENXQwpdHfKWnrAX4Q354hV7bHkHZN5EBxrioAVF7cGdz22wU48q5C14kzQFuBXwzv9GLxLtwcLWzuJqOLTVl+jOGBgecbSVMfUGqv99hVoxwANgWBAJBYnBFNO2dOlCbR/vfWD1tngy/geO8ZGyzarUr/wvQp2q7kI1MCBJ6QN0/Seti2/ABk3fe/9ASG7pmJJOkenXQ3LtBJD0AB4epB";

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;

    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor motorFL;
    private DcMotor motorFR;
    double sm = 1;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables targets = null;
    private WebcamName webcamName = null;

    private boolean targetVisible = false;
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();

    @Override
    public void runOpMode() {
        motorBL = hardwareMap.get(DcMotor.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotor.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotor.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotor.class, "motorFR"); // Motor Front-Right

        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);


        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;

        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        allTrackables.addAll(targets);

        identifyTarget(0, "Blue Storage", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall", halfTile, halfField, mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage", -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall", halfTile, -halfField, mmTargetHeight, 90, 0, 180);

        final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        chassis.start();
        vuforiaThread.start();
        while (!isStopRequested()) {

        }
    }

    private final Thread chassis = new Thread(() -> {
        while (!isStopRequested()) {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x * 1.5;
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
            } else if (gamepad1.right_bumper) {
                sm = 5;
                POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
            } else {
                sm = 0.5;
                POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
            }
            if(gamepad1.a){
                Log.d("Vuforia data,", "delimitare");
            }
        }
    });

    public Thread vuforiaThread = new Thread(() -> {
        targets.activate();
        while (!isStopRequested()) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    Log.d("Vuforia data,", "Visible Target," + trackable.getName());
                    targetVisible = true;
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            if (targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                Log.d("Vuforia data,", "Pos (inches)," + String.format("%.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch));
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                Log.d("Vuforia data,", "Rot (deg)," + String.format("%.0f, %.0f, %.0f",
                        rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle));
            } else {
                Log.d("Vuforia data,", "Visible Target," + "none");
            }
            waitMillis(100);
        }
        targets.deactivate();
    });

    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public void POWER(double df1, double sf1, double ds1, double ss1) {
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
    public void waitMillis(int t){
        double lastTime = System.currentTimeMillis();
        while(lastTime + t > System.currentTimeMillis()){}
    }
}
