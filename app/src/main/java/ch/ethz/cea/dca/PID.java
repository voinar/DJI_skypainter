package ch.ethz.cea.dca;

public class PID {

    private float kp;
    private float ki;
    private float kd;

    public PID(float kp, float ki, float kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public float getOutput() {
        return(0);
    }

    public void setGoal(float goal) {

    }

    public void setCurrent(float current) {

    }
}
