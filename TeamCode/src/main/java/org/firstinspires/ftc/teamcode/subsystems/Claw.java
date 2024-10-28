package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw extends SubsystemBase {
    private Servo servo;
    private ServoStates currentState;
    private final ElapsedTime switchTimer = new ElapsedTime(); // Timer for switch delay

    public enum ServoStates {
        OPEN(0.4),
        CLOSE(0.65);

        private final double position;

        ServoStates(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    public Claw(HardwareMap h) {
        servo = h.get(Servo.class, "Claw Servo");
        currentState = ServoStates.CLOSE;  // Default initial state
        servo.setPosition(currentState.getPosition()); // Set initial position
        switchTimer.reset();  // Initialize the timer
    }

    public void open() {
        setPosition(ServoStates.OPEN);
    }

    public void close() {
        setPosition(ServoStates.CLOSE);
    }

    public void setPosition(ServoStates state) {
        currentState = state;
        servo.setPosition(state.getPosition());
    }

    public void switchState() {
        // Only allow state switch if 700 milliseconds have passed
        if (switchTimer.milliseconds() >= 700) {
            if (currentState == ServoStates.OPEN) {
                close();
            } else {
                open();
            }
            switchTimer.reset();  // Reset the timer after switching state
        }
    }

    public boolean isOpen() {
        return currentState == ServoStates.OPEN;
    }

    public void clawPosition(double position) {
        servo.setPosition(position);
    }

    public double getPosition() {
        return servo.getPosition();
    }
}
