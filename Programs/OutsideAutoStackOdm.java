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
public class OutsideAutoStackOdm extends LinearOpMode {
    
    
    // Declares all motors
    DcMotorEx ShooterMotor1;
    DcMotor IntakeMotor;
    DcMotor IntakeMotor2;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    
    // Declares all Servos
    Servo ShooterServo;
    CRServo LeftServo;
    CRServo RightServo;
    CRServo IntakeServo;
    CRServo IntakeServo2;
    Servo GripperServo;
    Servo SideServo;
    Servo SideServo2;
    
    // Drive Motor Power Variables
    double leftFrontpower;
    double rightFrontpower;
    double leftBackpower;
    double rightBackpower;
    
    // Drive Input variables
    double leftsticky;
    double leftstickx;
    double rightstickx;
    double r;
    double robotangle;
    double rightX;
    
    // Drive Adjustment Variables
    double divide;
    double straightP;
    
    // Determines time of stack detection
    double visionReadTime;
    
    // Variables that track the angle of the robot
    double angle;
    String angleVal;
    
    // Variables for tracking the state of the shooter motor
    int currentCounts;
    int lastCounts;
    
    // Setup modifying PID values
    PIDFCoefficients pid;
    
    // Load TensorFlowData
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    
    // Prepare a timer
    ElapsedTime timer;
    
    // Setup Variables for voting
    double close;
    double middle;
    double far;
    int guess;
    
    // Prepare to use the gyro
    Acceleration gravity;
    BNO055IMU imu;
    Orientation angles;
    
    // Prepare to use Vuforia and TensorFlow
    private static final String VUFORIA_KEY = " AWFgCSD/////AAABmYkyQ5CPl0oxgJ1ax3vYAbqHDTIleFfJqDw8oca/v28OosWAbIHMkNSwkFuFnM7FPUcXM9sqqdHfFdyMulVLNQyAVUlboelnnXfdw3EkqFCQcF0q6EoJydb2+fJE8fWNLGOrvxZm9rkSX0NT9DVdE6UKfyc/TVpYTYaLegPitiLRpvG4P2cHsHhtUQ48LCuuPN2uFdC1CAJ6YRYtc7UMiTMZw8PyCKM1tlcG6v4dugoERLcoeX2OVA9eFJ2w89/PNK7rzNsLmo4OugTh3bztARq6S7gl+Q/DbscZ3/53Vg+1N4eIXZh/LJwJK6ZJxetftvcXBHi9j9f9T6/ghhY0szUzLmAoKlAO+0XXebOtXKad ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    
    // Store the constants for this autonomous
    public OutsideAutoStackOdm() {
        straightP = 0.075;
        visionReadTime = 0.1;
        currentCounts = 0;
        lastCounts = 0;
        timer = new ElapsedTime();
        far = 0.0;
    }
    
    // Adjustable Variables for alternate paths
    boolean startLeft = false;
    boolean shootStack = true;
    boolean parkFirst = true;
    double startWait = 2;

    public void runOpMode() throws InterruptedException {
        // Map all Motors
        ShooterMotor1 = hardwareMap.get(DcMotorEx.class, "SM1");
        IntakeMotor2 = hardwareMap.get(DcMotor.class, "IM2");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IM1");
        leftFront = hardwareMap.dcMotor.get("LF");
        rightFront = hardwareMap.dcMotor.get("RF");
        leftBack = hardwareMap.dcMotor.get("LB");
        rightBack = hardwareMap.dcMotor.get("RB");
        
        // Map all Servos
        ShooterServo = hardwareMap.get(Servo.class, "s1");
        GripperServo = hardwareMap.get(Servo.class, "GS");
        LeftServo = hardwareMap.get(CRServo.class, "LS");
        RightServo = hardwareMap.get(CRServo.class, "RS");
        IntakeServo = hardwareMap.get(CRServo.class, "IS");
        IntakeServo2 = hardwareMap.get(CRServo.class, "IS2");
        SideServo = hardwareMap.get(Servo.class, "SS");
        SideServo2 = hardwareMap.get(Servo.class, "SS2");
       
        // Set the mode and zero power behavior of motors
        ShooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Setup and apply PIDF to the shooter motor 
        final PIDFCoefficients newPIDF = new PIDFCoefficients(1000.0, 10.0, 0.0, 14.3);
        ShooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        pid = ShooterMotor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Initialize the gyro
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.update();
        
        // Create the timer
        timer = new ElapsedTime();
        
        // Initialize the vision system 
        initializeVision();
        
        // Let the drivers know that the robot is ready
        telemetry.addData("ROBOT IS READY", "PRESS START TO BEGIN");
        telemetry.update();
        
        // Start when the play button is pressed
        waitForStart();
        
        // Detect the amount of rings in the starting stack
        determineNumberOfRings();
        timer.reset();
        timer.startTime();
        if (tfod != null) {
            tfod.shutdown();
        }
        
        // Drives to the launch line and shoots
        ShooterMotor1.setPower(-0.61);
        SideServo2.setPosition(1.0);
        moveStraight('f', 17035, 0.0, 0.6);
        waiting(1.0);
        pointTurn('l',7.5,.4);
        shootThreeTimes(0.25);
        pointTurn('r',0,.4);
        moveStraight('b', 8500, 0.0, 0.8);
        IntakeServo.setPower(1.0);
        IntakeServo2.setPower(-1.0);
        IntakeMotor.setPower(-1.0);
        IntakeMotor2.setPower(1.0);
        waiting(0); // normal: 0 8373: 0 92: 5
        strafe('l', 6000, 0.5, 0);
            

            // Shoot the stack
            if (guess == 2) {
                moveStraight('f', 2300, 0.0, 0.2);
                waiting(3.5); // normal: 1.5 8373: 3.5 92: 3.5
                shootThreeTimes(0.25);
                moveStraight('f', 7125, 0.0, 0.4);
                waiting(1);
                shootThreeTimes(0.25);
            }
            if (guess == 1) {
                waiting(5); // normal: 0 8373: 5 92: 0
                moveStraight('f', 2500, 0.0, 0.2);
                waiting(1.5);
                moveStraight('f', 8000, 0.0, 0.4);
                waiting(.5);
                shootThreeTimes(.25);
            }
            if (guess == 0){
            waiting(1);
            }
                  if (guess == 2) {
            // Place the wobble goal
            moveStraight('f', 15020, 0.0, 0.9);
            strafe('r', 3245, 0.5, 0.0);
            SideServo2.setPosition(0.0);
            waiting(.5);

            // Return behind the line and shoot "bonus shots"
            strafe('l', 2820, 0.8, 0);
            moveStraight('b', 15720, -2.0, 0.95);
            shootThreeTimes(0.25);

            // Park on the line
            moveStraight('f', 1875, 0.0, 0.95);
        }
        if (guess == 1) {
            // Place the wobble goal
            pointTurn('l', 85, .4);
            strafe('r', 8260, .5, 90);
            strafe('l', 940, 0.5, 90);
            SideServo2.setPosition(0.0);

            // Park on the line
            strafe('l', 2820, 0.5, 90);
            pointTurn('r', 0, 0.4);
        }
        if (guess == 0) {
            // Sweep the field for extra rings
            moveStraight('f', 26500, 0.0, 0.9);
            waiting(.5);
            moveStraight('b', 16475, -2, 0.95);

            // Shoot "bonus rings"
            shootThreeTimes(0.25);

            // Place the wobble goal
            moveStraight('f', 2125, 0.0, 0.95);
            strafe('r', 4425, 0.5, 0.0);
            strafe('l', 1175, 0.5, 0.0);
            SideServo2.setPosition(0.0);

            // Park on the line
            strafe('l', 2350, 0.5, 0.0);
        }

      
        
         
        
    }
     // A method for initializing Vuforia
    private void initVuforia() {
        final VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = " AWFgCSD/////AAABmYkyQ5CPl0oxgJ1ax3vYAbqHDTIleFfJqDw8oca/v28OosWAbIHMkNSwkFuFnM7FPUcXM9sqqdHfFdyMulVLNQyAVUlboelnnXfdw3EkqFCQcF0q6EoJydb2+fJE8fWNLGOrvxZm9rkSX0NT9DVdE6UKfyc/TVpYTYaLegPitiLRpvG4P2cHsHhtUQ48LCuuPN2uFdC1CAJ6YRYtc7UMiTMZw8PyCKM1tlcG6v4dugoERLcoeX2OVA9eFJ2w89/PNK7rzNsLmo4OugTh3bztARq6S7gl+Q/DbscZ3/53Vg+1N4eIXZh/LJwJK6ZJxetftvcXBHi9j9f9T6/ghhY0szUzLmAoKlAO+0XXebOtXKad ";
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    // A method for initializing TensorFlow
    private void initTfod() {
        final int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        final TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        (tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia))
                .loadModelFromAsset("UltimateGoal.tflite", new String[] { "Quad", "Single" });
    }
    // Methods used for reading gyro input
    String formatAngle(final AngleUnit angleUnit, final double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(final double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    // A method for moving the robot straight based on direction, encoders, the angle to hold, and motor power
    void moveStraight(final char fb, final int encoderCount, final double holdAngle, final double motorPower) {
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.startTime();
        timer.reset();
        while (timer.time() < 0.25) {
        }
        if (fb == 'f') {
            while (rightBack.getCurrentPosition() < encoderCount) {
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
                telemetry.addData("Encoder Used:", leftFront.getCurrentPosition());
                telemetry.addData("Encoder Not Used:", rightBack.getCurrentPosition());
                telemetry.addData("Encoder Strafe:", rightFront.getCurrentPosition());
                telemetry.update();
                
            }
        } else {
            while (rightBack.getCurrentPosition() > -encoderCount) {
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
    
    // A method for point turning based on direction, the angle to turn to, and motor power
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
        } else {
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
    // A method for initializing all vision 
    void initializeVision() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.0, 1.78);
        }
    }
   
    // A method that detects the number of rings in the stack
    void determineNumberOfRings() {
        timer.reset();
        timer.startTime();
        while (timer.time() < visionReadTime && opModeIsActive()) {
            final List<Recognition> updatedRecognitions = (List<Recognition>) tfod.getRecognitions();
            close = 0.5;
            for (final Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel() == "Quad") {
                    ++far;
                } else {
                    ++middle;
                }
            }
        }
        if (far > middle && far > close) {
            guess = 2;
        } else if (middle > close) {
            guess = 1;
        } else {
            guess = 0;
        }
    }
    void shootOneTime(final double waitTime) {
        timer.reset();
        timer.startTime();
        ShooterServo.setPosition(1.0);
        while (timer.time() < waitTime && opModeIsActive()) {
        }
        ShooterServo.setPosition(0.5);
        timer.reset();
        timer.startTime();
        while (timer.time() < 0.25 && opModeIsActive()) {
        }
        ShooterServo.setPosition(1.0);
    }

    // A method that shoots three rings
    void shootThreeTimes(final double waitTime) {
        timer.reset();
        timer.startTime();
        ShooterServo.setPosition(1.0);
        while (timer.time() < waitTime) {
        }
        ShooterServo.setPosition(0.5);
        timer.reset();
        timer.startTime();
        while (timer.time() < 0.25) {
        }
        ShooterServo.setPosition(1.0);
        timer.reset();
        timer.startTime();
        while (timer.time() < waitTime) {
        }
        ShooterServo.setPosition(0.5);
        timer.reset();
        timer.startTime();
        while (timer.time() < 0.25) {
        }
        ShooterServo.setPosition(1.0);
        timer.reset();
        timer.startTime();
        while (timer.time() < waitTime) {
        }
        ShooterServo.setPosition(0.5);
        timer.reset();
        timer.startTime();
        while (timer.time() < 0.25) {
        }
        ShooterServo.setPosition(1.0);
    }
   
    // A method that prepares the robot to pick up a wobble goal
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
   
    // A method that picks up a wobble goal
    void wobblePickUp() {
        GripperServo.setPosition(1.0);
        timer.startTime();
        timer.reset();
        while (timer.time() < 1.0) {
        }
        timer.startTime();
        timer.reset();
        while (timer.time() < 0.6) {
            RightServo.setPower(-1.0);
            LeftServo.setPower(-1.0);
        }
        RightServo.setPower(0.0);
        LeftServo.setPower(0.0);
    }
    // A method that implements a pause
    void waiting(final double waittime) {
        timer.startTime();
        timer.reset();
        while (timer.time() < waittime) {
        }
    }
    
    // A method that allows the robot to strafe based on direction, encoder counts, motor power, and an angle to hold
    void strafe(final char lr, final int encoderCounts, final double motorPower, final double holdAngle) {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.startTime();
        timer.reset();
        while (timer.time() < 0.25) {
        }
        if (lr == 'l') {
            leftFront.setPower(-motorPower);
            leftBack.setPower(motorPower);
            rightFront.setPower(-motorPower);
            rightBack.setPower(motorPower);
            while (rightFront.getCurrentPosition() > -encoderCounts) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                leftFront.setPower(-motorPower + (angle - holdAngle) * 0.075);
                leftBack.setPower(motorPower + (angle - holdAngle) * 0.075);
                rightFront.setPower(-motorPower + (angle - holdAngle) * 0.075);
                rightBack.setPower(motorPower + (angle - holdAngle) * 0.075);
            }
        }
        if (lr == 'r') {
            int lowestValue = 0;
            leftFront.setPower(motorPower);
            leftBack.setPower(-motorPower);
            rightFront.setPower(motorPower);
            rightBack.setPower(-motorPower);
            while (rightFront.getCurrentPosition() < encoderCounts) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                /*telemetry.addData("LF:", leftFront.getCurrentPosition());
                telemetry.addData("RF:", rightFront.getCurrentPosition());
                telemetry.addData("LB:", leftBack.getCurrentPosition());
                telemetry.addData("RB:", rightBack.getCurrentPosition());
                telemetry.addData("Encoder", IntakeMotor.getCurrentPosition());
                telemetry.addData("Angle", angle);
                telemetry.update();*/
                leftFront.setPower(motorPower + (angle - holdAngle) * 0.075);
                leftBack.setPower(-motorPower + (angle - holdAngle) * 0.075);
                rightFront.setPower(motorPower + (angle - holdAngle) * 0.075);
                rightBack.setPower(-motorPower + (angle - holdAngle) * 0.075);
            }
        }
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
    }
}
