package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import selenium
@TeleOp
public class FullRobot extends OpMode {

    // Declare all motors
    DcMotorEx ShooterMotor1;
    DcMotor IntakeMotor;
    DcMotor IntakeMotor2;
    DcMotor  leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    // Declare all servos
    Servo ShooterServo;
    CRServo LeftServo;
    CRServo RightServo;
    CRServo IntakeServo;
    CRServo IntakeServo2;
    Servo GripperServo;
    Servo SideServo;
    Servo SideServo2;
    
    // P Value for strafing
    double straafeP = .05;

    // Motor power variables
    double leftFrontpower;
    double rightFrontpower;
    double leftBackpower;
    double rightBackpower;

    // Drive input variables
    double leftsticky;
    double leftstickx;
    double rightstickx;
    double r;
    double robotangle;
    double rightX;

    // Drive adjustment variables
    double divide;
    double motorPower;

    // Variables for toggling actions
    boolean shooteron;
    boolean isPressed;
    boolean intakeon;
    boolean isPressed2;
    boolean Gripperon;
    boolean isPressed3;
    boolean isPressed4;
    boolean isPressed55;
    boolean shooterlow;
    boolean intakereverse;

    // Variables for tracking the state of the shooter motor
    int currentCounts = 0;
    int lastCounts = 0;

    // Variables for tracking the angle of the robot
    double angle;
    String angleVal;

    // Setup modifying PID values
    PIDFCoefficients pid;

    // Prepare a timer
    ElapsedTime timer;

    // Declare the imu 
    BNO055IMU imu;

    // Prepare to use the gyro
    Orientation angles;
    
  



    @Override
    public void init() {
        // Map all motors
        ShooterMotor1 = hardwareMap.get(DcMotorEx.class, "SM1");
        IntakeMotor2 = hardwareMap.get(DcMotor.class, "IM2");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IM1");
        leftFront = hardwareMap.dcMotor.get("LF");
        rightFront = hardwareMap.dcMotor.get("RF");
        leftBack = hardwareMap.dcMotor.get("LB");
        rightBack = hardwareMap.dcMotor.get("RB");

        // Map all servos
        ShooterServo = hardwareMap.get(Servo.class, "s1");
        GripperServo = hardwareMap.get(Servo.class,"GS");
        LeftServo = hardwareMap.get(CRServo.class,"LS");
        RightServo = hardwareMap.get(CRServo.class,"RS");
        IntakeServo = hardwareMap.get(CRServo.class,"IS");
        IntakeServo2 = hardwareMap.get(CRServo.class,"IS2");
        SideServo = hardwareMap.get(Servo.class,"SS");
        SideServo2 = hardwareMap.get(Servo.class,"SS2");

        // Set modes of motors
        ShooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Initialize boolean values
        shooteron = false;
        isPressed = false;
        intakeon = false;
        isPressed2 = false;
        Gripperon = false;
        isPressed3 = false;
        isPressed4 = false;
        isPressed55 = false;
        shooterlow = false;
        intakereverse = false;

        // Setup the shooter motor
        PIDFCoefficients newPIDF = new PIDFCoefficients(1000,10,0,14.3);
        ShooterMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, newPIDF);
        pid = ShooterMotor1.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Create the timer
        timer = new ElapsedTime();
        
    }
       
    @Override
    public void loop() {
        // Print shooter motor data
        telemetry.addData("isPIDMode?", ShooterMotor1.getMode().isPIDMode());
        telemetry.addData("p", pid.p );
        telemetry.addData("i", pid.i );
        telemetry.addData("d", pid.d );
        telemetry.addData("f", pid.f );
        telemetry.addData("Motor Power", ShooterMotor1.getPower());
        telemetry.update(); 

        // Brings robot to starting orientation
        if (gamepad1.dpad_up) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            String angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
            double angle = Float.parseFloat(angleVal);
            if (angle > 5){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                leftBack.setPower(.4);
                leftFront.setPower(.4);
                rightBack.setPower(.4);
                rightFront.setPower(.4);
            }
            else if (angle < -5) {
                leftBack.setPower(-.4);
                leftFront.setPower(-.4);
                rightBack.setPower(-.4);
                rightFront.setPower(-.4);

            }
            else {
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }
        // Strafes robot along starting orientation     
        } else if (gamepad1.right_trigger > .5 || gamepad1.left_trigger > .5) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            String angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
            double angle = Float.parseFloat(angleVal);
    
        
            if (gamepad1.right_trigger > .5){
                
                leftstickx = -1;
                leftsticky = 0;
                rightstickx = -straafeP*angle;
                
            }
            else if (gamepad1.left_trigger > .5){
                leftstickx = 1;
                leftsticky = 0;
                rightstickx = -straafeP*angle;
            }
            // Uses values to determine motor powers 
            r = Math.hypot(leftstickx, leftsticky);
            robotangle = Math.atan2(leftsticky, leftstickx) - Math.PI / 4;
            rightX = rightstickx;
            leftFrontpower = (r * Math.cos(robotangle)) * (2 / Math.sqrt(2)) + rightX;
            leftBackpower = (r * Math.sin(robotangle)) * (2 / Math.sqrt(2)) + rightX;
            rightFrontpower = (r * Math.sin(robotangle)) * (2 / Math.sqrt(2)) - rightX;
            rightBackpower = (r * Math.cos(robotangle)) * (2 / Math.sqrt(2)) - rightX;
            if (leftFrontpower > 1 || leftFrontpower < -1) {
                divide = Math.abs(leftFrontpower);
                leftFrontpower = leftFrontpower / divide;
                leftBackpower = leftBackpower / divide;
                rightFrontpower = rightFrontpower / divide;
                rightBackpower = rightBackpower / divide;
            } else if (leftBackpower > 1 || leftBackpower < -1) {
                divide = Math.abs(leftBackpower);
                leftFrontpower = leftFrontpower / divide;
                leftBackpower = leftBackpower / divide;
                rightFrontpower = rightFrontpower / divide;
                rightBackpower = rightBackpower / divide;
            } else if (rightFrontpower > 1 || rightFrontpower < -1) {
                divide = Math.abs(rightFrontpower);
                leftFrontpower = leftFrontpower / divide;
                leftBackpower = leftBackpower / divide;
                rightFrontpower = rightFrontpower / divide;
                rightBackpower = rightBackpower / divide;
            } else if (rightBackpower > 1 || rightBackpower < -1) {
                divide = Math.abs(rightBackpower);
                leftFrontpower = leftFrontpower / divide;
                leftBackpower = leftBackpower / divide;
                rightFrontpower = rightFrontpower / divide;
                rightBackpower = rightBackpower / divide;
            }
            
            // Applies motor powers
            leftFront.setPower(-leftFrontpower);
            rightFront.setPower(rightFrontpower);
            leftBack.setPower(-leftBackpower);
            rightBack.setPower(rightBackpower);

        } else {
            // Robot control if not strafing 
            leftstickx = -gamepad1.left_stick_x;
            leftsticky = gamepad1.left_stick_y;
            rightstickx = -gamepad1.right_stick_x;

            // Uses values to determine motor powers 
            r = Math.hypot(leftstickx, leftsticky);
            robotangle = Math.atan2(leftsticky, leftstickx) - Math.PI / 4;
            rightX = rightstickx;
            leftFrontpower = (r * Math.cos(robotangle)) * (2 / Math.sqrt(2)) + rightX;
            leftBackpower = (r * Math.sin(robotangle)) * (2 / Math.sqrt(2)) + rightX;
            rightFrontpower = (r * Math.sin(robotangle)) * (2 / Math.sqrt(2)) - rightX;
            rightBackpower = (r * Math.cos(robotangle)) * (2 / Math.sqrt(2)) - rightX;
            if (leftFrontpower > 1 || leftFrontpower < -1) {
                divide = Math.abs(leftFrontpower);
                leftFrontpower = leftFrontpower / divide;
                leftBackpower = leftBackpower / divide;
                rightFrontpower = rightFrontpower / divide;
                rightBackpower = rightBackpower / divide;
            } else if (leftBackpower > 1 || leftBackpower < -1) {
                divide = Math.abs(leftBackpower);
                leftFrontpower = leftFrontpower / divide;
                leftBackpower = leftBackpower / divide;
                rightFrontpower = rightFrontpower / divide;
                rightBackpower = rightBackpower / divide;
            } else if (rightFrontpower > 1 || rightFrontpower < -1) {
                divide = Math.abs(rightFrontpower);
                leftFrontpower = leftFrontpower / divide;
                leftBackpower = leftBackpower / divide;
                rightFrontpower = rightFrontpower / divide;
                rightBackpower = rightBackpower / divide;
            } else if (rightBackpower > 1 || rightBackpower < -1) {
                divide = Math.abs(rightBackpower);
                leftFrontpower = leftFrontpower / divide;
                leftBackpower = leftBackpower / divide;
                rightFrontpower = rightFrontpower / divide;
                rightBackpower = rightBackpower / divide;
            }
            
            // Applies motor powers
            leftFront.setPower(-leftFrontpower);
            rightFront.setPower(rightFrontpower);
            leftBack.setPower(-leftBackpower);
            rightBack.setPower(rightBackpower);

            // Print driver motor data
            telemetry.addData("leftFront", leftFrontpower);
            telemetry.addData("rightFront", rightFrontpower);
            telemetry.addData("leftBack", leftBackpower);
            telemetry.addData("rightBack", rightBackpower);
            telemetry.update();
        }

        // Toggles shooter motor to high goal setting
        if (gamepad2.x) {
            shooterlow = false;
            if(isPressed == false) {
                isPressed = true;
                if (shooteron == false) {
                    shooteron = true;
                    ShooterMotor1.setPower(-.61);
                } else {
                    ShooterMotor1.setPower(0);
                    shooteron = false;
                }
            }
        // Toggles shooter motor to power shot / mid goal setting
        } else if (gamepad2.dpad_down) {
            shooteron = false;
            if(isPressed55 == false) {
                isPressed55 = true;
                if (shooterlow == false) {
                    shooterlow = true;
                    ShooterMotor1.setPower(-.52);
                } else {
                    ShooterMotor1.setPower(0);
                    shooterlow = false;
                }
            }
        }
        else {
            isPressed = false;
            isPressed55 = false;
        }

        // Record motor speed to log for later analysis
        if(shooteron) {
            if(timer.time()>0.1){
                currentCounts = ShooterMotor1.getCurrentPosition();
                System.out.println((currentCounts-lastCounts)/timer.time());
                lastCounts = currentCounts;
                timer.reset();
            
            }
            
        } 

        // Toggles intake 
        if (gamepad1.a) {
            if(isPressed2 == false) {
                isPressed2 = true;
                if (intakeon == false) {
                    intakeon = true;
                    IntakeMotor.setPower(-1);
                    IntakeMotor2.setPower(1);
                    IntakeServo.setPower(1);
                    IntakeServo2.setPower(-1);
                    

                } else {
                    IntakeMotor.setPower(0);
                    IntakeMotor2.setPower(0);
                    IntakeServo.setPower(0);
                    IntakeServo2.setPower(0);
                    intakeon = false;
                }
            }
        
        // Toggles intake (reversed)
        } else if (gamepad2.y) {
            if(isPressed4 == false) {
                isPressed4 = true;
                if (intakereverse == false) {
                    intakereverse = true;
                    IntakeMotor.setPower(1);
                    IntakeMotor2.setPower(-1);
                    IntakeServo.setPower(-1);
                    IntakeServo2.setPower(1);
                } else {
                    IntakeMotor.setPower(0);
                    IntakeMotor2.setPower(0);
                    IntakeServo.setPower(0);
                    IntakeServo2.setPower(0);
                    intakereverse = false;
                }
            }
            
        } else {
            isPressed2 = false;
            isPressed4 = false;
        }
        
        // Shoots rings
        if (gamepad2.a) {
            ShooterServo.setPosition(.5);
        } else {
            ShooterServo.setPosition(1);
        }
        
        // Bring wobble goal arm up
        if(gamepad2.left_bumper) {
            LeftServo.setPower(-1);
            RightServo.setPower(-1);
        // Bring wobble goal arm down
        } else if (gamepad2.right_bumper) {
            LeftServo.setPower(1);
            RightServo.setPower(1);
        } else {
            LeftServo.setPower(0);
            RightServo.setPower(0);
        }
        
        // Toggles gripper position
        if (gamepad2.b) {
            if(isPressed3 == false) {
                isPressed3 = true;
                if (Gripperon == false) {
                    Gripperon = true;
                    GripperServo.setPosition(1);
                 
                } else {
                    GripperServo.setPosition(0.45);
                    Gripperon = false;
                }
            }
        } else {
            isPressed3 = false;
        }
        
        // Release for autonomous wobble goal latches
        if(gamepad2.dpad_right){
            SideServo.setPosition(1);
        } else {
            SideServo.setPosition(0);
        }
        
        if(gamepad2.dpad_left){
            SideServo2.setPosition(1);
        } else {
            SideServo2.setPosition(0);
        }
       
        
        
  

    }
    
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    
}

