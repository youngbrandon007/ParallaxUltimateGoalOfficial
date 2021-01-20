package org.firstinspires.ftc.teamcode.teamcode.prometheus.opmodes.auto;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Collector;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.TrackerWheels;
import org.firstinspires.ftc.teamcode.teamcode.prometheus.robot.WobbleArm;

import java.util.List;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {

    DriveTrain dt;
    Shooter shooter;
    TrackerWheels tw;

    //Collector collector;
    //WobbleArm wobbleArm;

    double collectorSpeed;
    boolean on = false;
    boolean click = false;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime stateTime = new ElapsedTime();

    enum State{
        DriveForward, Scan, MoveToShoot, Shoot,
        aDrive, aBack,
    }

    State state;

    //Camera
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "Ae+3PJ3/////AAABmZ++KArd6ULIncvgnh8X3ptJvwWkLf1RfwQj4HHP4vOgeJfYtwM+Vv15UT/UoJXxLS66cmC0hWS6+2DTBkedwKF1J96AqwqhGnDPZiBiLyWaWA1G2HUAEco5vMDNzfW79UmNTWrTRNNuma4LhssPRkAc8zHDG/MTOBJgh5NSYotpY99czSMSoc+niTh/AyUVlH2xYyoNuLO0a48k+J9GlSs0N8PDrBXbPvgUKJ6Hclmf/iTsXvJWQdBg3vMXEtvCql7oXQAJbcjjdXaOvDtNmJaTD8HuqtZ+TBhlKlLDDPJkc2mNQbX6VaSznpmDxjeL81WBY4qwR9vhGTmIvwtX60go2cbd//pdt3/H2UTyRbZN";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    double targetSpeed = 0;
    Boolean shooterOn = false;

    String stack = null;
    int stackNullCount = 0;
    int stackQuadCount = 0;
    int stackSingleCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        dt = new DriveTrain(this, true);
        dt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tw = new TrackerWheels(this);


        shooter = new Shooter(this);

        //collector = new Collector(this);

        //wobbleArm = new WobbleArm(this, true);

        //camera
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();

        shooter.indexerUp();
        shooter.shooterPusherBack();
        time.reset();

        state = State.DriveForward;

        shooter.target = 1600;

        tw.reset(dt.backLeft.getCurrentPosition(),dt.frontLeft.getCurrentPosition(),dt.frontRight.getCurrentPosition());
        while(opModeIsActive()){
            if(time.milliseconds() > 50){
                tw.update(dt.backLeft.getCurrentPosition(), dt.frontLeft.getCurrentPosition(), dt.frontRight.getCurrentPosition(), time.seconds());
                switch (state){
                    case DriveForward:
                        targetSpeed = (22 - tw.pos.x) * 3;  //22
                        targetSpeed = clip(targetSpeed, 30);
                        dt.xPID(tw.velocity, targetSpeed, tw.pos.angle.getDegrees(), 0);
                        if(Math.abs(22 - tw.pos.x) < 2) {  //22
                            dt.stop();
                            state = State.Scan;
                            stateTime.reset();
                        }

                        break;
                    case Scan:

                        if (tfod != null) {
                            // getUpdatedRecognitions() will return null if no new information is available since
                            // the last time that call was made.
                            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                            if (updatedRecognitions != null) {
                                telemetry.addData("# Object Detected", updatedRecognitions.size());
                                // step through the list of recognitions and display boundary info.
                                int i = 0;
                                stack = null;
                                for (Recognition recognition : updatedRecognitions) {
                                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                            recognition.getLeft(), recognition.getTop());
                                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                            recognition.getRight(), recognition.getBottom());
                                    stack = recognition.getLabel();
                                }

                                if(stack == null){
                                    stackNullCount++;
                                }else if(stack.equals("quad")){
                                    stackQuadCount++;
                                }else{
                                    stackSingleCount++;
                                }
                            }

                        }
                        shooterOn = true;

                        if(stateTime.seconds() > 1){
                            if(stackNullCount > stackQuadCount && stackNullCount > stackSingleCount){
                                stack = null;
                            }else if(stackQuadCount > stackNullCount && stackQuadCount > stackSingleCount){
                                stack = "quad";
                            }else{
                                stack = "single";
                            }
                            state = State.MoveToShoot;
                        }
                        break;

                    case MoveToShoot:
                        targetSpeed = (58 - tw.pos.x) * 3;
                        targetSpeed = clip(targetSpeed, 30);
                        dt.xPID(tw.velocity, targetSpeed, tw.pos.angle.getDegrees(), 0);
                        if(Math.abs(58 - tw.pos.x) < 2) {
                            dt.stop();
                            stateTime.reset();
                            state = State.Shoot;
                        }

                        break;
                    case Shoot:

                        if(stateTime.seconds() > 2.0) {
                            shooter.shooterPusherBack();
                            if(stack == null){
                                state = State.aDrive;
                            }
                        }if(stateTime.seconds() > 1.6){
                            shooter.shooterPusherOut();
                        }else if(stateTime.seconds() > 1.2){
                            shooter.shooterPusherBack();
                        }else if(stateTime.seconds() > 0.8){
                            shooter.shooterPusherOut();
                        }else if(stateTime.seconds() > 0.4){
                            shooter.shooterPusherBack();
                        }else{
                            shooter.shooterPusherOut();
                        }
                        break;
                    case aDrive:
                        targetSpeed = (-90 - tw.pos.angle.getDegrees()) * -1;
                        targetSpeed = clip(targetSpeed, 20);
                        dt.xPIDwRotation(tw.velocity, targetSpeed, -0.8);
                        if(Math.abs(-90 - tw.pos.angle.getDegrees()) < 5) {
                            dt.stop();
                            stateTime.reset();
                            state = State.aBack;
                            shooterOn = false;
                        }

                        break;
                    case aBack:
                        targetSpeed = (0 - tw.pos.y) * -3;
                        targetSpeed = clip(targetSpeed, 20);
                        dt.xPID(tw.velocity, targetSpeed, tw.pos.angle.getDegrees(), -90);
                        if(Math.abs(0 - tw.pos.y) < 2) {
                            dt.stop();
                            stateTime.reset();
                        }
                        break;
                }
                if(shooterOn) {
                    shooter.update(time.seconds());
                }
                else {
                    shooter.shooter.setPower(0);
                }

                time.reset();

                telemetry.addData("Stack", stack);
                telemetry.addData("stack null", stackNullCount);
                telemetry.addData("stack single", stackSingleCount);
                telemetry.addData("stack quad", stackQuadCount);
                telemetry.addData("state", state);
                telemetry.addData("target Value", shooter.target);
                telemetry.update();
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    double clip(double val, double clipto){
        if(val > clipto){
            val = clipto;
        }else if(val < - clipto){
            val = -clipto;
        }
        return val;
    }
}