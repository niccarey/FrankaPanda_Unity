using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

// joint: create from Articulation Body

namespace KDLsolver
{

    public class JointArray
    {
        public int length;

        public Vector<float> Create(int ArrayLength) {
            length = ArrayLength;
            return Vector<float>.Build.Dense(ArrayLength);
        }

    }

   
}
