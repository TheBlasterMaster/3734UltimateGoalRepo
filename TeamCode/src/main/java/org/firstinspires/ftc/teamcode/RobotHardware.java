package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.Arrays;

public class RobotHardware {
    // ------ HARDWARE
    public DcMotor frontLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backLeftDrive;
    public DcMotor backRightDrive;
    public DcMotor intakeMotor;
    public DcMotor intakeMotor2;
    public DcMotorEx flyWheel;
    public DcMotor wobbleArm;

    public Servo trigger;
    public Servo gripper;
    public Servo funnel;
    public Servo funnel_trigger;
    public Servo funnel2; 
    
    public ColorSensor top;
    public ColorSensor bottom;

    BNO055IMU imu;
    // ------




    public void init(HardwareMap hwmp) {
        HardwareMap hardwareMap = hwmp;
        //Obtain Hardware Refrences
        //--------------------------

        //DRIVE TRAIN
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");

        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        //INTAKE, SHOOTING, WOBBLE GOAL
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intake2");
        flyWheel = (DcMotorEx)(hardwareMap.get(DcMotor.class, "flywheel"));
        wobbleArm = hardwareMap.get(DcMotor.class, "wobble");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);
        wobbleArm.setDirection(DcMotor.Direction.FORWARD);

        //SERVOS
        funnel_trigger = hardwareMap.get(Servo.class, "funnel_trigger");

        trigger = hardwareMap.get(Servo.class, "trigger");
        gripper = hardwareMap.get(Servo.class, "wobble_servo");
        funnel = hardwareMap.get(Servo.class, "funnel");
        funnel2 = hardwareMap.get(Servo.class, "funnel2");
        funnel_trigger = hardwareMap.get(Servo.class, "funnel_trigger");

        gripper.setDirection(Servo.Direction.REVERSE);
        funnel.setDirection(Servo.Direction.FORWARD);
        funnel_trigger.setDirection(Servo.Direction.FORWARD);
        funnel2.setDirection(Servo.Direction.REVERSE);
        
        //COLOR SENSORS
        top = hardwareMap.get(ColorSensor.class, "top");
        bottom = hardwareMap.get(ColorSensor.class, "bottom");

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //=========================
        
        //Set Run Modes
        flyWheel.setMode(DcMotor.RunMode.RESET_ENCODERS);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set PID Coeffecients of FlyWheel Motor
        PIDCoefficients pidOrig = flyWheel.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients newPID = new PIDCoefficients(30,3,0);
        flyWheel.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODERS, newPID);
        
    }
}