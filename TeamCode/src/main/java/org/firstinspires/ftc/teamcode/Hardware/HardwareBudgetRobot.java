package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *   Budget: Mecanum/Holometric Drive
 */

public class HardwareBudgetRobot {

    /* Constants */

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    /* Public Opmode Members */
    public DcMotor motor1,motor2, motor3, motor4, motor5;
    public Servo servo1;
    BNO055IMU imu;


    public HardwareBudgetRobot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        /* Drive Hardware */
        motor1 = myOpMode.hardwareMap.get(DcMotor.class, "Motor 1");
        motor2 = myOpMode.hardwareMap.get(DcMotor.class, "Motor 2");
        motor3 = myOpMode.hardwareMap.get(DcMotor.class, "Motor 3");
        motor4 = myOpMode.hardwareMap.get(DcMotor.class, "Motor 4");
        motor5 = myOpMode.hardwareMap.get(DcMotor.class, "Motor 5");


        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        motor5.setPower(0);
        /* Intake Hardware */
        
        /* Servos */
        servo1 = myOpMode.hardwareMap.get(Servo.class, "Servo 1");
    }


    /* Intake Methods */

}
