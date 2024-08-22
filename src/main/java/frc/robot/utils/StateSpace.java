// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

/** Add your docs here. */
public class StateSpace {
    double ks = 0.01;
    double kv = 0.11;
    double ka = 0.1;


    Matrix<N2,N2> A = MatBuilder.fill(Nat.N2(), Nat.N2(), 0 ,0, -ks/ka, -kv/ks);
    Matrix<N2,N1> B = MatBuilder.fill(Nat.N2(), Nat.N1(),0, 1/ka);
    Matrix<N1,N2> C = MatBuilder.fill(Nat.N1(), Nat.N2(), 0, 1);
    Matrix<N1,N1> D = MatBuilder.fill(Nat.N1(), Nat.N1(), 0);

    LinearSystem<N2,N1,N1> ls = new LinearSystem<>(A,B,C,D); 
    LinearQuadraticRegulator<N2,N1,N1> lqr = new LinearQuadraticRegulator<>(ls,VecBuilder.fill(0.1,2), VecBuilder.fill(8),0.02);
    double kp = lqr.getK().get(0, 0);
    double ki = lqr.getK().get(0, 0);
}
