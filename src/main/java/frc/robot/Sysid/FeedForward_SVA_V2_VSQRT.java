package frc.robot.Sysid;

public class FeedForward_SVA_V2_VSQRT {
    double KS;
    double KV;
    double KA;
    double KV2;
    double KVSQRT;

    public FeedForward_SVA_V2_VSQRT(double KS, double KV, double KA, double KV2, double KVSQRT) {
        this.KA = KA;
        this.KV = KV;
        this.KS = KS;
        this.KV2 = KV2;
        this.KVSQRT = KVSQRT;
    }

    public double calculate(double v, double lastV) {
        return KS*Math.signum(v) + KV*v + KA*(v-lastV) + KV2*v*v + KVSQRT*Math.sqrt(Math.abs(v));
    }
}