/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;


/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot

 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")
public class CrazyDriver extends OpMode{

    /* Declare OpMode members. */
    OurRobot robot      = new OurRobot(); // use the class created to define a Pushbot's hardware
    int catCounter;
    boolean catOn=false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        robot.spinnerMotor.setPower(0.0); // Drivers did not like spinner starting right away
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftMotorPower;
        double rightMotorPower;
        boolean bButton;
        boolean aButton;
        boolean right_bumper;
        boolean left_bumper;
        boolean x_button;
        boolean y_button;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        leftMotorPower = -gamepad1.left_stick_y;
        rightMotorPower = -gamepad1.right_stick_y;

        robot.leftDrive.setPower(leftMotorPower);
        robot.rightDrive.setPower(rightMotorPower);

        //if y is pressed, itll go to half position. when it isnt pressed, it go to max position
        y_button = gamepad2.y;
        if (y_button) {
            robot.pushBeaconServo.setPosition(1);
        } else robot.pushBeaconServo.setPosition(0.5);

        //if x is pressed, itll go to half position. when it isnt pressed, it go to max position
        x_button = gamepad2.x;
        if (x_button) {

            robot.pushBallServo.setPosition(0.5);

        } else robot.pushBallServo.setPosition(1.0);


        //press to reverse the spinner
        if (gamepad2.left_bumper) left_bumper = true;
        else left_bumper = false;
        if (left_bumper) {
            if (robot.spinnerMotor.getPower() != 0) {
                robot.spinnerMotor.setPower(0);
            } else robot.spinnerMotor.setPower(-1);

        }

        // if right bumper is pushed, if the spinner is on, turn it off. If spinner is off, turn it on.
        right_bumper = gamepad2.right_bumper;
        if (right_bumper) {
            if (robot.spinnerMotor.getPower() > 0) {
                robot.spinnerMotor.setPower(0);
            } else robot.spinnerMotor.setPower(1);
        }


        // If the aButton button is pushed, throw the ball.
        aButton = gamepad2.a;
        if (aButton) {
            robot.throwerMotor.setPower(-1);
            catCounter=0;
            catOn=true;
        }
        else robot.throwerMotor.setPower(0);
        if (catOn) catCounter++;

        telemetry.addData("catOn",catOn);
        telemetry.addData("catCounter",catCounter);


        if ((catOn) && (catCounter==3000))
        {

            robot.throwerMotor.setPower(1);
            catOn=false;

        }


        // If the bButton button is pushed, throw the ball.
        bButton=gamepad2.b;
        if (bButton)
        {

            try {
                throwIt(1, //power = full
                        20 //percentage of full rotation to swing.
                );
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }


        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", leftMotorPower);
        telemetry.addData("right", "%.2f", rightMotorPower);
        updateTelemetry(telemetry);
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.spinnerMotor.setPower(0); // Stop the spinner motor

    }


    // Activate the thrower arm at a given power and time.
    public void throwIt(double throwPower, // how hard to throw it
                        int percentRotation // percentage of full rotation to move the throwing arm.
    ) throws InterruptedException {

        int newThrowPosition;
        int discretePositions = 1440; // discrete number of positions for the motor

        // Determine new target position, and pass to motor controller
        newThrowPosition = robot.throwerMotor.getCurrentPosition() + percentRotation / 100 * discretePositions;
        robot.throwerMotor.setTargetPosition(newThrowPosition);

        // Turn On RUN_TO_POSITION
        robot.throwerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.throwerMotor.setPower(throwPower);
        telemetry.addData("Running to %7d :%7d", newThrowPosition);

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (robot.throwerMotor.isBusy()) {

            // Display it for the driver.
            telemetry.addData("Running at %7d :%7d", robot.throwerMotor.getCurrentPosition());
            telemetry.update();

            // Allow time for other processes to run.
            wait(5); // wait 5 milliseconds

        }

        // Stop throwing
        robot.throwerMotor.setPower(0);
        telemetry.update();
    }
    }

