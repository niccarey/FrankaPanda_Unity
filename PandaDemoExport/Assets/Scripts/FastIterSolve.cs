using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

/* KDL-RR Solver:
 *
 * At each iteration we send:
 *     current joint positions (q_seed)
 *     target position
 *
 * Method:
 *     Calculate cartesian pose of end effector given forward kinematics
 *     Calculate error vector between current pose and target pose
 *     If error vector is sufficiently small, stop
 *     
 *     Calculate Jacobian from current joint values
 *
 *     Next joint position is calculated from qN = qS + J-1.p_err
 *     Now check that qN is reachable given joint and velocity limits
 *     for each joint
 *         if qNi exceeds joint limit, set to limit (qNi')
 *         if (qNi-qSi)/dt exceeds velocity limit, calculate maximum qNi' that can be reached in the same direction
 *         if there is a joint coupling or constraint based on an existing joint's position, calculate qNi' from that constraint
 *         
 *     If qN' - qS < epsilon for some chosen value, we have reached a local minima
 *     and need to generate a new seed (which also needs to be reachable from our current seed)
 *     New seed generation:
 *         get a randomised set of joint positions within reachable distance from current joint positions, based on velocity limits
 *         check for joint limits
 *         check for constraints
 *         set qN' = new seed
 *
 *     otherwise send joint commands and go round again.
 *
 * See TRAC-IK for improvements to this algorithm
 * 
 */

/* notes, pre-code:
 * We might need to spin off threads to accomplish solutions in real time
 * KDL (which I am somewhat adapting) does not include explicit inversion methods for jacobians
 * rather it does iterative sweeps to build matrix manipulations, on the grounds this has better numerical conditioning.
 * So to include an inversion you would instead add an explicit iterative numerical solver as a solver class, rather than
 * manipulating abstract matrix elements.
 *
 * to get an inverted Jacobian suggest using the SVD inverse method here for speed, other methods provided for completeness.
 * See chainIKSolver_wdls for a weighted optimization solver implementation (to add later as a test for solving redundant manipulators)
 * Note that one way KDL speeds up all processes is because having a formalized SOLVER class allows us to preallocate the
 * necessary solver components, which makes a big difference in C++. Not sure if this adds significant acceleration in C#.
 *
 * TODO: use NL solver for redundant manipulators (? maybe? might be slower)
 * TODO: fix joint count, behaviour for parallel children
 * TODO: Implement [Frame], [Pose] and [Twist] classes to make solvers more modular, less clunky, lets us use internal class methods
 * TODO: how to handle spherical joints? FABRIK is better for this
 * TODO: Make a Utility class that contains methods used across multiple classes, eg. skew-symmetric matrix builder
 * TODO: Check for angular wrapping (modulo pi) 
 * TODO: check for gradient divergence and implement (RR? some kind of reseed) if error is not converging.
*/

public class FastIterSolve
{

    public Jacobian chainJacobian;
    public float converge_eps = 1e-4f; // equiv to about 5mm in tool space error, which is not really good enough
    // ideally we should have a well-conditioned solver that can handle down to 0.1mm accuracy (will this conflict with soft joint control though?)
    public float joint_eps = 1e-6f;
    public Vector<float> p_err;
    public float prev_pose_cost = 100;

    public Vector<float> CalcPoseError(Matrix4x4 CurrentPose, Matrix4x4 goalPose)
    {
        // Convert end effector to 6-vector pose representation in appropriate frame
        Vector<float> calcP_err = Vector<float>.Build.Dense(6);

        Vector3 positionError = GetPosition(goalPose) - GetPosition(CurrentPose);

        Quaternion rotationError = CurrentPose.rotation * Quaternion.Inverse(goalPose.rotation);

        /* There is the chance that we end up with some ambiguity in pose
         * if the path to the solution passes through a complete 180 rotation
         * Usually we solve this by enforcing a constraint on the quaternion representation
         * but we are using an (inherently ambiguous) euler rotation instead.
         * Forcing a higher weight on the rotation error leads to lack of convergence even for
         * physically plausable situations.
         */
        Vector3 rotationEuler = rotationError.eulerAngles;

        calcP_err[0] = positionError.x;
        calcP_err[1] = positionError.y;
        calcP_err[2] = positionError.z;
        calcP_err[3] = Mathf.Sin(Mathf.Deg2Rad * rotationEuler.x); // rotationErrVec.x;
        calcP_err[4] = Mathf.Sin(Mathf.Deg2Rad * rotationEuler.y); // rotationErrVec.y;
        calcP_err[5] = Mathf.Sin(Mathf.Deg2Rad * rotationEuler.z); // rotationErrVec.z;


        if (System.Single.IsInfinity(calcP_err[3]))
        {
            rotationError = Quaternion.identity;
            rotationEuler = rotationError.eulerAngles;
            calcP_err[3] = Mathf.Sin(Mathf.Deg2Rad * rotationEuler.x); // rotationErrVec.x;
            calcP_err[4] = Mathf.Sin(Mathf.Deg2Rad * rotationEuler.y); // rotationErrVec.y;
            calcP_err[5] = Mathf.Sin(Mathf.Deg2Rad * rotationEuler.z); // rotationErrVec.z;
        }

        return calcP_err;
    }

    public Vector3 GetPosition(Matrix4x4 transformMatrix)
    {
        return new Vector3(transformMatrix.m03, transformMatrix.m13, transformMatrix.m23);
    }

    public Vector3 GetAxisAngleVec(Quaternion poseRot)
    {
        float axAngle;
        Vector3 axis;
        poseRot.ToAngleAxis(out axAngle, out axis);
        return new Vector3(axAngle * axis[0], axAngle * axis[1], axAngle * axis[2]);
    }


    public Vector<float> JointLimitConstraints(Chain currentChain, Vector<float> q_state)
    {
        Vector<float> q_out = Vector<float>.Build.Dense(q_state.Count);
        q_state.CopyTo(q_out);
        for (int i = 0; i < q_state.Count; i++)
        {
            if (q_out[i] < currentChain.jointLimits[i][0]) { q_out[i] = currentChain.jointLimits[i][0]; }
            if (q_out[i] > currentChain.jointLimits[i][1]) { q_out[i] = currentChain.jointLimits[i][1]; }
        }
        return q_out;
    }

    public Vector<float> JointVelocityConstraints(Chain currentChain, Vector<float> q_state, Vector<float> q_update)
    {
        Vector<float> q_out = Vector<float>.Build.Dense(q_update.Count);
        q_update.CopyTo(q_out);

        for (int i = 0; i < q_update.Count; i++)
        {
            if (Mathf.Abs(q_update[i] - q_state[i]) / Time.fixedDeltaTime > currentChain.jointVelLimit[i])
            {
                float vdes = Mathf.Abs((q_update[i] - q_state[i])) / Time.fixedDeltaTime;
                q_out[i] = q_state[i] + (q_update[i] - q_state[i]) * currentChain.jointVelLimit[i] / vdes;
            }
        }
        return q_out;

    }

    public float GetNewSeed(float seedAngle, float velocityLimit)
    {
        float minAngle = seedAngle -  2.0f*velocityLimit * Time.fixedDeltaTime;
        float maxAngle = seedAngle +  2.0f*velocityLimit * Time.fixedDeltaTime;

        return Random.Range(minAngle, maxAngle);
    }

    public float IterCostFunction(Vector<float> errorVec, int flag = 0)
    {
        // TODO extend this with different options for weighted cost functions etc depending on flag

        // case: default weight scaling, conditioned for cartesian elements being in metres and orientation in radians.
        float[] costWeighting = { 1, 1, 1, 1,1,1 };

        return errorVec * (Matrix<float>.Build.DenseOfDiagonalArray(costWeighting) * errorVec);

    }

    public Matrix<float> SkewMatrix(Vector3 inputVec)
    {
        // returns the Levi-Civita permutation of a vector (skew-symmetric matrix)
        Matrix<float> skewS = Matrix<float>.Build.DenseIdentity(3);
        skewS.SetColumn(0, new float[] { 0.0f, inputVec[2], -inputVec[1] });
        skewS.SetColumn(1, new float[] { -inputVec[2], 0.0f, inputVec[0] });
        skewS.SetColumn(2, new float[] { inputVec[1], -inputVec[0] , 0.0f});
        return skewS;
    }

    //This method is poor. 
    /* public Vector<float> CartToJnt_GS(Chain projectChain, Vector<float> q_seed, Matrix4x4 goalPose, out bool targetReachedFlag)
    {
        p_err = CalcPoseError(projectChain.base2EETransform, goalPose);
        targetReachedFlag = false;

        float pose_cost = IterCostFunction(p_err);

        if (pose_cost < converge_eps) // ignore rotation error for now
        {
            targetReachedFlag = true;
            return q_seed;
        }


        Jacobian chainJacobian = projectChain.BuildJacobian(q_seed, 1.0f);
        chainJacobian.SVDPseudoInverse();

        // TO AVOID SINGULARITIES: we can add a damping term in the manner of Gauss-Seidel

        // method: use transpose, rather than inverse
        // Calculate Db = JT.dX
        // Calculate damped square Jacobian A = (JT.J + deltaI)
        // Use Gauss-Seidel to solve for dTheta in A. dTheta = Db
        // how does htis play with redundancy? task-based prioritisation?

        Vector<float> delta_state = chainJacobian.inverse * p_err;
        Matrix<float> condJac = (chainJacobian.inverse * chainJacobian.value + 0.001f * Matrix<float>.Build.DenseIdentity(7));

        // depends on the Jacobian diagonals being much larger than other elements: poorly conditioned jacobians may fail
        Vector<float> q_update = q_seed;
        for (int i = 0; i < q_seed.Count; i++)
        {
            float delta_q_i = (0.001f*delta_state[i] - condJac.Row(i).DotProduct(q_seed)) / condJac[i, i];
            q_update[i] += delta_q_i;
        }

        // check for limits
        q_update = JointLimitConstraints(projectChain, q_update);

        q_update = JointVelocityConstraints(projectChain, q_seed, q_update);

        return q_update;


    }*/

        // Chain braking method: Assume the first three joints largely decide the manipulator position, while the remainder
        // constrain the orientation. (note sure how valid this is in this case but anyway)
        // We consider theta1-3 when minimizing EE position, exclude theta4-5 (as these will be required to 'join' the broken chain'), then solve orientation using theta6-7
        // note: this will leave the manipulator underconstrained, how do the authors deal with that?


        // Bensalah (building off Siciliano et al), propose a jacobian transpose method instead of a pseudo-inverse.
    // But while this guarantees (* sort of) a solution, this may lead to loss of precision? Is this purely to avoid singularities, or is there some convergence advantage?
    // Unclear.

    public Vector<float> CartToJnt(Chain projectChain, Vector<float> q_seed, Matrix4x4 goalPose, Vector<float> q_init, out bool targetReachedFlag,  params float[] weights)
    {

        p_err = CalcPoseError(projectChain.base2EETransform, goalPose);

        targetReachedFlag = false;
        float pose_cost = IterCostFunction(p_err);
        if (pose_cost < converge_eps) 
        {
            Debug.Log(p_err);
            targetReachedFlag = true;
            return q_seed;
        }

        Jacobian chainJacobian = projectChain.BuildJacobian(q_seed, 1.0f);

        chainJacobian.SVDPseudoInverse();
        // Rotation solution is sometimes moving in the wrong direction --> can't converge.
        // This is not consistent behaviour --> I don't think anything is necessarily wrong with the Jacobian, but something in the process is screwy.

        // Try task-based prioritisation rather than weight-based prioritisation

        // CHIAVERINI: (use this if we don't actually care very much about orientation)
        // Split Jacobian into two task-based components. J1 handles position, J2 handles orientation.
        Vector<float> T1_err = Vector<float>.Build.DenseOfVector(p_err);
        T1_err.ClearSubVector(3, 3);
        Vector<float> T2_err = Vector<float>.Build.DenseOfVector(p_err); // why is this negative?
        T2_err.ClearSubVector(0, 3);

        float alpha = 0.1f;
        if (1 / pose_cost > 1000.0f) { alpha = 1.0f; } // switch to newtonian method? faster but could encourage singularities

        // Jacobian error checking:
        Matrix<float> expandedInverse = Matrix<float>.Build.DenseOfMatrix(chainJacobian.value * chainJacobian.inverse);
        Vector<float> errorCheck = (Matrix<float>.Build.DenseIdentity(p_err.Count) - expandedInverse) * p_err;

        Matrix<float> jointProject;
        Vector<float> q_update;

        // Damping the Jacobian helps some issues but doesn't completely eliminate non-convergence issues near singularities.
        
        if (errorCheck.SumMagnitudes() > 10e-6)
        {
            //Debug.Log("Using damped Jacobian");
            //Debug.Log(errorCheck.SumMagnitudes()); // if this is over 10e-7 then we are probably heading into a singularity
            float dampingFactor = 0.01f * (float)errorCheck.SumMagnitudes(); // Chan and Lawrence, 1988

            Matrix<float> RobustJacInv = chainJacobian.RobustInverse(dampingFactor);

            // we are near a singularity: use damped robust jacobian
            jointProject = (Matrix<float>.Build.DenseIdentity(q_seed.Count) - RobustJacInv * chainJacobian.value);
            //q_update = q_seed + alpha * (RobustJacInv * T1_err) + jointProject * (RobustJacInv * T2_err);
            q_update = q_seed + alpha * (RobustJacInv * p_err);

        }

        else
        {
            // Build null-space projection term:
            jointProject = (Matrix<float>.Build.DenseIdentity(q_seed.Count) - chainJacobian.inverse * chainJacobian.value);
            //q_update = q_seed + alpha * (chainJacobian.inverse * T1_err) + jointProject * (chainJacobian.inverse * T2_err);
            q_update = q_seed + alpha * (chainJacobian.inverse * p_err);
        }


        // Can use Meredith and Maddock to speed things up (if error > e for some e, iteratively halve p_err
        // this lets us use large movements when appropriate, drills down to a finer granularity in non-linear areas.
        // not particularly helpful for slow end convergence.

        // check for limits
        q_update = JointLimitConstraints(projectChain, q_update);

        // check for wrapping
        // q_update = JointWrapConstraints(projectChain, q_update);
        // nB; for a well-defined set of robot joints and limits, we shouldn't hit discontinuities


        // Do joint coupling checks and update joints if necessary
        // q_update = ExternalJointConstraints(projectChain, q_update);

        // Check for local minima
        // should also check for increase in error?
        if ((q_update - q_seed).L2Norm() < joint_eps)
        {
            Debug.Log("local min, reseed ...");
            // reseed using velocity bounds // actually this is a dumb way to reseed, should try a different method.
            for (int i = 0; i < q_seed.Count; i++)
            {
                q_update[i] = GetNewSeed(q_init[i], projectChain.jointVelLimit[i]);
            }
            q_update = JointLimitConstraints(projectChain, q_update);
            
            // q_update = ExternalJointConstraints(projectChain, q_update);
            // should the seeding be recursive, in case we don't move outside the local minima catchment?
        }
        else if (pose_cost - prev_pose_cost > 0.01)
        {
            // error is increasing, not converging (this might not always be bad but...)
            Debug.Log("local min, reseed ...");
            // reseed using velocity bounds // actually this is a dumb way to reseed, should try a different method.
            for (int i = 0; i < q_seed.Count; i++)
            {
                q_update[i] = GetNewSeed(q_init[i], projectChain.jointVelLimit[i]);
            }
            q_update = JointLimitConstraints(projectChain, q_update);
            this.prev_pose_cost = 1000.0f;

        }
        this.prev_pose_cost = pose_cost;
        // Check for p_err divergence and reseed

        return q_update;

    }

    public Vector<float> LMCartToJoint(Chain projectChain, Vector<float> q_seed, Matrix4x4 goalPose)
    {
        // Levenberg-Marquadt algorithm, for better conditioning?
        p_err = CalcPoseError(goalPose, projectChain.base2EETransform); // Get dP

        return p_err;

    }


}


