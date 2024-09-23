package frc.robot.Sysid;

public class FeedForward_SVA {
    double KS;
    double KV;
    double KA;

    public FeedForward_SVA(double KS, double KV, double KA) {
        this.KA = KA;
        this.KV = KV;
        this.KS = KS;
    }

    public FeedForward_SVA(double[] K) {
        this(K[0], K[1], K[2]);
    }

    public double calculate(double v, double lastV) {
        return KS*Math.signum(v) + KV*v + KA*(v-lastV);
    }
}