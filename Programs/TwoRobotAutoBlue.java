package org.firstinspires.ftc.teamcode;

import java.util.Iterator;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TwoRobotAutoBlue extends LinearOpMode
{
    DcMotorEx ShooterMotor1;
    DcMotor IntakeMotor;
    DcMotor IntakeMotor2;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    Servo ShooterServo;
    CRServo LeftServo;
    CRServo RightServo;
    CRServo IntakeServo;
    CRServo IntakeServo2;
    CRServo IntakeServo3;
    Servo GripperServo;
    Servo SideServo;
    Servo SideServo2;
    double leftFrontpower;
    double rightFrontpower;
    double leftBackpower;
    double rightBackpower;
    double leftsticky;
    double leftstickx;
    double rightstickx;
    double r;
    double robotangle;
    double rightX;
    double divide;
    double straightP;
    double visionReadTime;
    double angle;
    String angleVal;
    int currentCounts;
    int lastCounts;
    PIDFCoefficients pid;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    ElapsedTime timer;
    double close;
    double middle;
    double far;
    int guess;
    Acceleration gravity;
    BNO055IMU imu;
    Orientation angles;
    private static final String VUFORIA_KEY = " AWFgCSD/////AAABmYkyQ5CPl0oxgJ1ax3vYAbqHDTIleFfJqDw8oca/v28OosWAbIHMkNSwkFuFnM7FPUcXM9sqqdHfFdyMulVLNQyAVUlboelnnXfdw3EkqFCQcF0q6EoJydb2+fJE8fWNLGOrvxZm9rkSX0NT9DVdE6UKfyc/TVpYTYaLegPitiLRpvG4P2cHsHhtUQ48LCuuPN2uFdC1CAJ6YRYtc7UMiTMZw8PyCKM1tlcG6v4dugoERLcoeX2OVA9eFJ2w89/PNK7rzNsLmo4OugTh3bztARq6S7gl+Q/DbscZ3/53Vg+1N4eIXZh/LJwJK6ZJxetftvcXBHi9j9f9T6/ghhY0szUzLmAoKlAO+0XXebOtXKad ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public TwoRobotAutoBlue() {
        straightP = 0.075;
        visionReadTime = 0.1;
        currentCounts = 0;
        lastCounts = 0;
        timer = new ElapsedTime();
        far = 0.0;
    }
    
    boolean startLeft = false; 
    boolean shootStack = true;
    boolean parkFirst = true;
    double startWait = 0;
    
    public void runOpMode() throws InterruptedException {
        ShooterMotor1 = hardwareMap.get(DcMotorEx.class, "SM1");
        IntakeMotor2 = hardwareMap.get(DcMotor.class, "IM2");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IM1");
        ShooterServo = hardwareMap.get(Servo.class, "s1");
        GripperServo = hardwareMap.get(Servo.class, "GS");
        LeftServo = hardwareMap.get(CRServo.class, "LS");
        RightServo = hardwareMap.get(CRServo.class, "RS");
        IntakeServo = hardwareMap.get(CRServo.class, "IS");
        IntakeServo2 = hardwareMap.get(CRServo.class, "IS2");
        SideServo = hardwareMap.get(Servo.class, "SS");
        SideServo2 = hardwareMap.get(Servo.class, "SS2");
        ShooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront = hardwareMap.dcMotor.get("LF");
        rightFront = hardwareMap.dcMotor.get("RF");
        leftBack = hardwareMap.dcMotor.get("LB");
        rightBack = hardwareMap.dcMotor.get("RB");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        final PIDFCoefficients newPIDF = new PIDFCoefficients(1000.0, 10.0, 0.0, 14.3);
        ShooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        pid = ShooterMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
        telemetry.update();
        timer = new ElapsedTime();
        initializeVision();
        telemetry.addData("ROBOT IS READY", "PRESS START TO BEGIN");
        telemetry.update();
        waitForStart();
        determineNumberOfRings();
        timer.reset();
        timer.startTime();
        if (tfod != null) {
            tfod.shutdown();
        }
        ShooterMotor1.setPower(-0.61);
        SideServo.setPosition(0.0);
        waiting(startWait);
        moveStraight('f', 1000, 0.0, 0.6);
        if (startLeft == true) {
        strafe('r',900,0.5,0);
        }
        else {
        strafe('l',1100,0.5,0);
        }
        if (shootStack == true) {
            if (guess == 2){
            moveStraight('f', 200, 0.0, 0.2);
            shootThreeTimes(0.5);
            IntakeServo.setPower(1.0);
            IntakeServo2.setPower(-1.0);
            IntakeMotor.setPower(-1.0);
            IntakeMotor2.setPower(1.0);
            moveStraight('f', 200, 0.0, 0.2);
            waiting(1.5);
            shootThreeTimes(0.5);
            moveStraight('f', 1400, 0.0, 0.4);
            waiting(1);
            shootThreeTimes(0.25);  
            }
            if (guess == 1){
            moveStraight('f', 300, 0.0, 0.2);
            timer.reset();
            timer.startTime();
            ShooterServo.setPosition(1.0);
            while (timer.time() < 0.5) {}
            ShooterServo.setPosition(0.5);
            timer.reset();
            timer.startTime();
            while (timer.time() < 0.25) {}
            ShooterServo.setPosition(1.0);
            IntakeServo.setPower(1.0);
            IntakeServo2.setPower(-1.0);
            IntakeMotor.setPower(-1.0);
            IntakeMotor2.setPower(1.0);
            moveStraight('f', 1500, 0.0, 0.4);
            waiting(1.0);
            shootThreeTimes(0.25);
            }
            if (guess == 0){
                moveStraight('f', 200, 0.0, 0.2);
                IntakeServo.setPower(1.0);
                IntakeServo2.setPower(-1.0);
                IntakeMotor.setPower(-1.0);
                IntakeMotor2.setPower(1.0);
                moveStraight('f', 1600, 0.0, 0.6);
                waiting(1.0);
                shootThreeTimes(0.25);
            }
            IntakeServo.setPower(0);
            IntakeServo2.setPower(0);
            IntakeMotor.setPower(0);
            IntakeMotor2.setPower(0);
            }
         else {
            IntakeServo.setPower(1.0);
            IntakeServo2.setPower(-1.0);
            IntakeMotor.setPower(-1.0);
            IntakeMotor2.setPower(1.0);
            moveStraight('f', 1800, 0.0, 0.6);
            waiting(1.0);
            shootThreeTimes(0.25);
              IntakeServo.setPower(0);
            IntakeServo2.setPower(0);
            IntakeMotor.setPower(0);
            IntakeMotor2.setPower(0);
            waiting(.25);
         }
           
        if (guess == 2) {
            moveStraight('f', 2350, 0.0, 0.9);
            waiting(0.25);
            strafe('l', 900, 0.5, 0.0);
            strafe('r', 300, 0.5, 0.0);
            SideServo.setPosition(1.0);        
            
        }
        if (guess == 1) {
            pointTurn('r',-85,.4);
            strafe('l',1400,.5,-90);
            strafe('r',200,0.5,-90);
            SideServo.setPosition(1.0);
            strafe('r',600,0.5,-90);
            pointTurn('l',-5,0.4);
        }
        if (guess == 0) {
            moveStraight('f', 400, 0.0, 0.75);
            strafe('l', 900, 0.5, 0.0);
            strafe('r', 200, 0.5, 0.0);
            SideServo.setPosition(1.0);
        }
        if (parkFirst == true ){
        if (guess == 2){
        strafe('r',1700,0.5,0);
        moveStraight('b',-2000,0.0,0.6);
        }
        if (guess == 1){
        strafe('r',1200,0.5,0);
        }
        if (guess == 0){
        strafe('r',1800,0.5,0);
        }
        }
        else {
          if (guess == 2){
        moveStraight('b',-2000,0.0,0.6);
        }
            
        }
    }
    
    private void initVuforia() {
        final VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = " AWFgCSD/////AAABmYkyQ5CPl0oxgJ1ax3vYAbqHDTIleFfJqDw8oca/v28OosWAbIHMkNSwkFuFnM7FPUcXM9sqqdHfFdyMulVLNQyAVUlboelnnXfdw3EkqFCQcF0q6EoJydb2+fJE8fWNLGOrvxZm9rkSX0NT9DVdE6UKfyc/TVpYTYaLegPitiLRpvG4P2cHsHhtUQ48LCuuPN2uFdC1CAJ6YRYtc7UMiTMZw8PyCKM1tlcG6v4dugoERLcoeX2OVA9eFJ2w89/PNK7rzNsLmo4OugTh3bztARq6S7gl+Q/DbscZ3/53Vg+1N4eIXZh/LJwJK6ZJxetftvcXBHi9j9f9T6/ghhY0szUzLmAoKlAO+0XXebOtXKad ";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    
    private void initTfod() {
        final int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        final TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        (tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)).loadModelFromAsset("UltimateGoal.tflite", new String[] { "Quad", "Single" });
    }
    
    String formatAngle(final AngleUnit angleUnit, final double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    
    String formatDegrees(final double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    
    void moveStraight(final char fb, final int encoderCount, final double holdAngle, final double motorPower) {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.startTime();
        timer.reset();
        while (timer.time() < 0.25) {}
        if (fb == 'f') {
            while (leftBack.getCurrentPosition() < encoderCount) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                final String angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                double angle = Float.parseFloat(angleVal);
                if (holdAngle - angle > 180.0 && holdAngle > 0.0 && angle < 0.0) {
                    final double dif = 180.0 - Math.abs(angle);
                    angle = 180.0 + dif;
                }
                leftFront.setPower(motorPower + (angle - holdAngle) * straightP);
                leftBack.setPower(motorPower + (angle - holdAngle) * straightP);
                rightFront.setPower(-motorPower + (angle - holdAngle) * straightP);
                rightBack.setPower(-motorPower + (angle - holdAngle) * straightP);
            }
        }
        else {
            while (leftBack.getCurrentPosition() > encoderCount) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                if (holdAngle - angle > 180.0 && holdAngle > 0.0 && angle < 0.0) {
                    final double dif2 = 180.0 - Math.abs(angle);
                    angle = 180.0 + dif2;
                }
                leftFront.setPower(-motorPower + (angle - holdAngle) * straightP);
                leftBack.setPower(-motorPower + (angle - holdAngle) * straightP);
                rightFront.setPower(motorPower + (angle - holdAngle) * straightP);
                rightBack.setPower(motorPower + (angle - holdAngle) * straightP);
            }
        }
        leftBack.setPower(0.0);
        leftFront.setPower(0.0);
        rightBack.setPower(0.0);
        rightFront.setPower(0.0);
    }
    
    void pointTurn(final char lr, final double targetAngle, final double motorPower) {
        if (lr == 'l') {
            while (angle < targetAngle) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                if (targetAngle == 180.0 && angle < 0.0) {
                    final double dif = 180.0 - Math.abs(angle);
                    angle = 180.0 + dif;
                }
                leftBack.setPower(-motorPower);
                leftFront.setPower(-motorPower);
                rightBack.setPower(-motorPower);
                rightFront.setPower(-motorPower);
            }
        }
        else {
            while (angle > targetAngle) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                leftBack.setPower(motorPower);
                leftFront.setPower(motorPower);
                rightBack.setPower(motorPower);
                rightFront.setPower(motorPower);
            }
        }
        leftBack.setPower(0.0);
        leftFront.setPower(0.0);
        rightBack.setPower(0.0);
        rightFront.setPower(0.0);
    }
    
    void initializeVision() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.0, 1.78);
        }
    }
    
    void determineNumberOfRings() {
        timer.reset();
        timer.startTime();
        while (timer.time() < visionReadTime && opModeIsActive()) {
            final List<Recognition> updatedRecognitions = (List<Recognition>)tfod.getRecognitions();
            close = 0.5;
            for (final Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel() == "Quad") {
                    ++far;
                }
                else {
                    ++middle;
                }
            }
        }
        if (far > middle && far > close) {
            guess = 2;
        }
        else if (middle > close) {
            guess = 1;
        }
        else {
            guess = 0;
        }
    }
    
    void shootThreeTimes(final double waitTime) {
        timer.reset();
        timer.startTime();
        ShooterServo.setPosition(1.0);
        while (timer.time() < waitTime) {}
        ShooterServo.setPosition(0.5);
        timer.reset();
        timer.startTime();
        while (timer.time() < 0.25) {}
        ShooterServo.setPosition(1.0);
        timer.reset();
        timer.startTime();
        while (timer.time() < waitTime) {}
        ShooterServo.setPosition(0.5);
        timer.reset();
        timer.startTime();
        while (timer.time() < 0.25) {}
        ShooterServo.setPosition(1.0);
        timer.reset();
        timer.startTime();
        while (timer.time() < waitTime) {}
        ShooterServo.setPosition(0.5);
        timer.reset();
        timer.startTime();
        while (timer.time() < 0.25) {}
        ShooterServo.setPosition(1.0);
    }
    
    void wobblePickUpPrep() {
        timer.reset();
        timer.startTime();
        while (timer.time() < 1.75) {
            RightServo.setPower(1.0);
            LeftServo.setPower(1.0);
        }
        RightServo.setPower(0.0);
        LeftServo.setPower(0.0);
        timer.startTime();
        timer.reset();
        GripperServo.setPosition(0.45);
    }
    
    void wobblePickUp() {
        GripperServo.setPosition(1.0);
        timer.startTime();
        timer.reset();
        while (timer.time() < 1.0) {}
        timer.startTime();
        timer.reset();
        while (timer.time() < 0.6) {
            RightServo.setPower(-1.0);
            LeftServo.setPower(-1.0);
        }
        RightServo.setPower(0.0);
        LeftServo.setPower(0.0);
    }
    
    void waiting(final double waittime) {
        timer.startTime();
        timer.reset();
        while (timer.time() < waittime) {}
    }
    
    void strafe(final char lr, final int encoderCounts, final double motorPower, final double holdAngle) {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.startTime();
        timer.reset();
        while (timer.time() < 0.25) {}
        if (lr == 'l') {
            leftFront.setPower(-motorPower);
            leftBack.setPower(motorPower);
            rightFront.setPower(-motorPower);
            rightBack.setPower(motorPower);
            while (leftBack.getCurrentPosition() < encoderCounts) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                leftFront.setPower(-motorPower + (angle - holdAngle) * 0.004);
                leftBack.setPower(motorPower + (angle - holdAngle) * 0.004);
                rightFront.setPower(-motorPower + (angle - holdAngle) * 0.004);
                rightBack.setPower(motorPower + (angle - holdAngle) * 0.004);
            }
        }
        if (lr == 'r') {
            leftFront.setPower(motorPower);
            leftBack.setPower(-motorPower);
            rightFront.setPower(motorPower);
            rightBack.setPower(-motorPower);
            while (leftBack.getCurrentPosition() > -encoderCounts) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                leftFront.setPower(motorPower + (angle - holdAngle) * 0.004);
                leftBack.setPower(-motorPower + (angle - holdAngle) * 0.004);
                rightFront.setPower(motorPower + (angle - holdAngle) * 0.004);
                rightBack.setPower(-motorPower + (angle - holdAngle) * 0.004);
            }
        }
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
    }
}