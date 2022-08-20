package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class cdcdcdddcdcdcdc extends LinearOpMode {
    private static final String VUFORIA_KEY = "AZE4VCz/////AAABmb4JroObBkAaoMAUdREOJfB0qeXdlSJIikrCfgHPgODESPjhm6+77xrN/bZDJ+D/OHBbiWcACYjEZq4YIW59s1ADEEh53nuQ3lSUXwK201KYKM28ZqLbFoiALrrQs0ruBE6QD8V5l6rhwBiEnSb9zf6YVwaunCsQW4rFUKpigaXJ9lphFJ9xVAOKGqFzHBZB6+tCWuD2430WkpEcJE/pOp/OAJyj/aTkekXKgMRUTxq3sLPKCGvUBPO8vNWXPy8tnGyiwL/Da77Kuke8CqqQS7HxAAnyq4we+cfNoascZfUnhHMEs2im8Ycw0g5aV0EhHhvRz//XH85rtWeYSrOHnS6yWtB8lAw1JHnFwntZnmvM";
    private VuforiaLocalizer vuforia;
    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();
    }
    private void initVuforia(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod(){

    }
}
