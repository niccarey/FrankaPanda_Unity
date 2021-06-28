using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class Jacobian
{

    public Matrix<float> value;
    public Matrix<float> inverse;
    public float eps = 1e-4f; // lower rounding limit

    // See Chain to build a list of jacobian vectors from a series of chain segments

    public Jacobian(Vector<float>[] JacArray)
    {
        value = Matrix<float>.Build.DenseOfColumnVectors(JacArray);
    }

    public Jacobian(Matrix<float> jacMatrix)
    {
        value = Matrix<float>.Build.DenseOfMatrix(jacMatrix);
    }

    public Vector<float> BuildColumn(Vector3 dx, Vector3 dtheta)
    {
        Vector<float> JacVec = Vector<float>.Build.Dense(6);
        JacVec[0] = dx.x;
        JacVec[1] = dx.y;
        JacVec[2] = dx.z;
        JacVec[3] = dtheta.x;
        JacVec[4] = dtheta.y;
        JacVec[5] = dtheta.z;

        return JacVec;
    }

    public void Transpose()
    {
        inverse = value.Transpose();
    }

    public void MoorePenroseInv()
    {
        inverse = value.PseudoInverse();
    }


    public void SVDPseudoInverse()
    {
        var svd_decomp = value.Svd(true);
        // Jacobian inverse can be calculated from V*S_recip*U'

        Vector<float> Sinv = svd_decomp.S.DivideByThis(1);
        for (int i = 0; i < Sinv.Count; i++) { if (Mathf.Abs(svd_decomp.S[i]) < eps) { Sinv[i] = 0; } }
        Matrix<float> temp_inverse = svd_decomp.W.Transpose();

        temp_inverse.SetDiagonal(Sinv);

        inverse = svd_decomp.VT.Transpose() * (temp_inverse * svd_decomp.U.Transpose());
    }

    public void GetSVDValues(out Vector<float> S, out Matrix<float> U, out Matrix<float> VT)
    {
        var svd_decomp = value.Svd(true);
        // Return S, U, transpose of T
        S = svd_decomp.S;
        U = svd_decomp.U;
        VT = svd_decomp.VT;

    }

    public Matrix<float> SVDPseudoInverse(Matrix<float> inputMatrix)
    {
        // overloaded method for general SVD-based matrix inversion.

        var svd_decomp = inputMatrix.Svd(true);
        // Jacobian inverse can be calculated from V*S_recip*U'

        Vector<float> Sinv = svd_decomp.S.DivideByThis(1);
        for (int i = 0; i < Sinv.Count; i++) { if (Mathf.Abs(svd_decomp.S[i]) < eps) { Sinv[i] = 0; } }
        Matrix<float> temp_inverse = svd_decomp.W.Transpose();

        temp_inverse.SetDiagonal(Sinv);
        return svd_decomp.VT.Transpose() * (temp_inverse * svd_decomp.U.Transpose());
    }

    public Matrix<float> RobustInverse(float dampingFactor)
    {
        // add damping factor to Jacobian near singularities
        Matrix<float> tempJac = value * value.Transpose() + (dampingFactor * Matrix<float>.Build.DenseIdentity(value.RowCount));
        return value.Transpose() * tempJac.Inverse(); // full rank so shouldn't need to use PS or MP
    }

    public Matrix<float> GeneralisedInverse(Matrix<float> inertiaMat)
    {
        Matrix<float> tempInvMatrix = (this.value * inertiaMat.Inverse() * this.value.Transpose());

        return inertiaMat.Inverse() * this.value.Transpose() * tempInvMatrix.Inverse();
    }

    public Matrix<float> TaskInertiaMatrix(Matrix<float> inertiaMat)
    {
        return (this.value * inertiaMat.Inverse() * this.value.Transpose()).Inverse();
    }

    public Matrix<float> GenInverseTranspose(Matrix<float> inertiaMat)
    {
        Matrix<float> taskInertia = TaskInertiaMatrix(inertiaMat);

        return taskInertia * this.value * inertiaMat.Inverse();
    }


    public Jacobian DeepCopy()
    {
        Jacobian returnJac = new Jacobian(this.value);
        if (this.inverse != null) { returnJac.inverse = Matrix<float>.Build.DenseOfMatrix(this.inverse); }
        return returnJac;
    }

    /*
    public Matrix<float> ExtendedJacInv()
    {
        // extended Jacobian method with optimisation

        // construct a full-rank jacobian by adding cost function conditions
        // to the baseline Jacobian
        // can use this to add soft closed-kinematic constraints, for example

        return extendedJacInverse()

    } */

}
