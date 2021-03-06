package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 * This class can be used to define all the specific hardware for our robot.
 * This class will be used for both modes of operation (teleop and autonomous).  Which means we only have to 
 *   change our robot configuration code in one place, not two.
 *  
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *  
 *
 * Motor channel:  Left  drive motor:        "leftDrive"
 * Motor channel:  Right drive motor:        "rightDrive"
 * Motor channel:  Manipulator drive motor:  "armMotor"
 * Servo channel:  Servo to open left claw:  "leftHand"
 * Servo channel:  Servo to open right claw: "rightHand"
 */
public class OurRobot
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  spinnerMotor  = null;
    public DcMotor  throwerMotor    = null;
    public Servo pushBeaconServo = null;
    public Servo pushBallServo = null;


public
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public OurRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize servos
        pushBallServo= hwMap.servo.get("pushBallServo");
        pushBeaconServo= hwMap.servo.get("pushBeaconServo");

        // Define and Initialize Motors
        leftDrive   = hwMap.dcMotor.get("leftDrive");
        rightDrive    = hwMap.dcMotor.get("rightDrive");
        throwerMotor = hwMap.dcMotor.get("throwerMotor");
        spinnerMotor = hwMap.dcMotor.get("spinnerMotor");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        throwerMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        spinnerMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        throwerMotor.setPower(0);
        spinnerMotor.setPower(0);

        // Set  these motors to run without encoders.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the thrower motor to run without an encoder
        throwerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void tryAwait(long millis)  // try to do a sleep, which should work...
    {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();

        }
    }

        public void throwTheBall(){
         throwerMotor.setPower(-1);//throw at full power for a half of a second
         tryAwait(200);
         throwerMotor.setPower(0.1);//go back down at 40% power for a half of a second
         tryAwait(500);
        }

        public void loadNextBall(){
            pushBallServo.setPosition(0.5);
            tryAwait(300);
            pushBallServo.setPosition(1.0);


        }

    public void ExecuteStep(double left, double right, double seconds, String description){
        // Step 1:  Drive forward for 2 seconds
        ElapsedTime stepTime = new ElapsedTime();
        SetMotorPowers(left,right);
        stepTime.reset();
        while (stepTime.seconds() < seconds) {
             Thread.yield();  // If other threads of work need a chance to do something, let them.
        }


    }

    private void SetMotorPowers(double left, double right){
        //telemetry.addData("power",  );
        leftDrive.setPower(left);
        rightDrive.setPower(right);

    }

}



