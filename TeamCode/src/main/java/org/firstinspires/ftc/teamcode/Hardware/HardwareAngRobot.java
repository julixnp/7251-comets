package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 *   Budget: Mecanum/Holometric Drive
 */

public class HardwareAngRobot {

    /* Constants */

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    /* Public Opmode Members */
    public DcMotor motor1,motor2, motor3, motor4, motorArm;

    public Servo servo3;
    public CRServo servo1, servo2;
    BNO055IMU imu;


    public HardwareAngRobot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        /* Drive Hardware */
        motor1 = myOpMode.hardwareMap.get(DcMotor.class, "motor1");
        motor2 = myOpMode.hardwareMap.get(DcMotor.class, "motor2");
        motor3 = myOpMode.hardwareMap.get(DcMotor.class, "motor3");
        motor4 = myOpMode.hardwareMap.get(DcMotor.class, "motor4");
        motorArm = myOpMode.hardwareMap.get(DcMotor.class, "arm");


        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        motorArm.setPower(0);
        /* Intake Hardware */
        
        /* Servos */
        servo1 = myOpMode.hardwareMap.get(CRServo.class, "Servo 1");
        servo2 = myOpMode.hardwareMap.get(CRServo.class, "Servo 2");
    }


    /* Intake Methods */

}
