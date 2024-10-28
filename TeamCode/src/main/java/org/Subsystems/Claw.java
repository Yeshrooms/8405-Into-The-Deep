import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo servo;
    private static final double OPEN_POSITION = 0.5;  
    private static final double CLOSE_POSITION = 0.0; 

    public Claw(Servo servo) {
        this.servo = servo;
    }

    public void open() {
        servo.setPosition(OPEN_POSITION);
    }

    public void close() {
        servo.setPosition(CLOSE_POSITION);
    }
    
    public double getPosition() {
        return servo.getPosition();
    }
}
