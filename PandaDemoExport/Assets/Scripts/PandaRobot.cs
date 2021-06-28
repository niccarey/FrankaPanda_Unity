using System;
using System.Collections;
using System.Threading;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;
using System.IO;

public class PandaRobot
{
   // Panda-specific constants
    readonly float[] maxJointarray = { 2.8973f, 1.7628f, 2.8973f, 3.0718f, 2.8973f, 0.0175f, 2.8973f };
    readonly float[] minJointarray = { -2.8973f, -1.7628f, -2.8973f, 0.0698f, -2.8973f, -3.7525f , -2.8973f };

    // upper triangle of inertia tensors for all bodies in articulation (row major form: Ixx, Ixy, Ixz, Iyy, Iyz, Izz)
    readonly float[][] inertiaArrays = new float[][]
    {
        new float[] { 7.0337e-01f, -1.3900e-04f, 6.7720e-03f, 7.0661e-01f, 1.9169e-02f, 9.1170e-03f },
        new float[] { 7.9620e-03f, -3.9250e-03f, 1.0254e-02f, 2.8110e-02f, 7.0400e-04f, 2.5995e-02f },
        new float[] { 3.7242e-02f, -4.7610e-03f, -1.1396e-02f, 3.6155e-02f, -1.2805e-02f, 1.0830e-02f },
        new float[] { 2.5853e-02f, 7.7960e-03f, -1.3320e-03f, 1.9552e-02f, 8.6410e-03f, 2.8323e-02f },
        new float[] { 3.5549e-02f, -2.1170e-03f, -4.0370e-03f, 2.9474e-02f, 2.2900e-04f, 8.6270e-03f },
        new float[] { 1.9640e-03f, 1.0900e-04f, -1.1580e-03f, 4.3540e-03f, 3.4100e-04f, 5.4330e-03f },
        new float[] { 1.2516e-02f, -4.2800e-04f, -1.1960e-03f, 1.0027e-02f, -7.4100e-04f, 4.8150e-03f }
    };

    readonly float[] massArray = { 2.92f, 2.74f, 2.74f, 2.38f, 2.38f, 2.74f, 1.55f, 0.54f };
    public float velocity = 40.0f; // joint velocity limiter

    // Gripper constants and limits
    private float gripper_open = 0.05f;
    private float gripper_shut = 0.028f;
    public float gripper_goal = 0.0f;
    public float GripDistance = 0.0f;


    // Panda robot parts:
    public Chain Articulation;
    public Vector<float> q_initial;

    // Select solver we want to use
    public FastIterSolve IKSolver = new FastIterSolve();

    // simulation-specific constants:
    public float maxCycles = 10000;

    // Attributes used or accessible by the libfranka robot instance
    //current end-effector pose can be accessed through articulation.base2EETransform.
    public Matrix4x4 last_goal_pose; // most recent DESIRED ee-pose in base frame

    // --- other end effector states
    // nominal end-effector pose in flange frame (apparently this is not accessible to libfranka)
    // end-effector pose in flange frame (by default identical to nominal pose but can be changed)
   
    public Matrix4x4 EE_T_K;  // Stiffness frame pose in end-effector frame (not sure exactly what this is)

    // --- Mass constants:
    // mass of the end effector
    // rotational inertia matrix of the end effector load with respect to center of mass (of ee or robot??)
    // center of mass of the end effector load with respect to flange frame.
    // mass of the external load (estimated from live parameters? or input by user?)
    // rotational inertia matrix of the external load with respect to center of mass
    // center of mass of the external load with respect to flange frame
    // Sum of the mass of the end effector and the external load. 
    // Combined rotational inertia matrix of the end effector load and the external load with respect to the center of mass.
    // Combined center of mass of the end effector load and the external load with respect to flange frame

    public Matrix<float> DynamicInertia;
    public Vector<float> GravityCompForces;
    public float[,,] ChrCo; // Christoffel coefficients
    public Matrix<float> CoriolisMatrix;

    // --- Elbow configuration info: this is the position (state) of the 3rd joint and the sign of the 4th joint. Not sure what we use this for
    // Elbow configuration, desired elbow configuration, commanded elbow configuration, commanded elbow velocity and acceleration

    // --- Joint torque sensor configuration -  needs some kind of torque sensor in articulation joint
    // Measured link-side joint torque sensor signals
    // Desired link-side joint torque sensor signals without gravity
    // Derivative of measured link-side joint torque sensor signals.

    // --- Joint state configuration
    // q can be obtained from Articulation.jointState
    public Vector<float> q_goal_state; // desired joint state
    // Vector<float> qd_goal_state; // desired joint velocites
    // Vector<float> qdd_goal_state; // desired joint acceleration
    public Vector<float> qd_state; // measured joint velocities
    public Vector<float> qdd_state; // measured joint acceleration

    // --- Contact and collision flags
    // each joint and each cartesian dimension (xyz, RPY) have associated contact flags which track contact state
    // each joint and each cartesian dimension have associated collision flags which are switched on after a collision (but do not reset themselves afterwards)

    // --- force and torque info
    // External torque, filtered. (is a joint state vector, not sure how this is estimated)
    // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame. Is a 6-state cartesian vector for (xyz, rpy)
    // Estimated external wrench(force, torque) acting on stiffness frame, expressed relative to the stiffness frame

    // --- Adidtional twists and poses
    //Desired end effector twist in base frame (6-element cartesian derivative vector)
    //Last commanded end effector pose of motion generation in base frame (transform matrix)
    //Last commanded end effector twist in base frame (6-element cartesian derivative vector)
    //Last commanded end effector acceleration in base frame.

    // --- Motor position and velocity - joint state vector containing motor commands


    // Workspace limit constants:
    float panda_lateral_radius = 855f;
    float panda_RHymax = 1190f;
    float panda_RHymin = -360f;
    float panda_LHzmin = -200f;
    float panda_LHyquadline = 600f;


    public PandaRobot(GameObject robotBase)
    {
        // TODO: need a general solver class and then we can construct the robot using a specific solver.

        Vector<float> pandaJointMin = Mathf.Rad2Deg * Vector<float>.Build.DenseOfArray(minJointarray);
        Vector<float> pandaJointMax = Mathf.Rad2Deg * Vector<float>.Build.DenseOfArray(maxJointarray);


        Vector3 objectBuffer = new Vector3(0.1f, 0.0f, 0.0f); // include an offset to account for finger length

        Articulation = new Chain(robotBase, pandaJointMin, pandaJointMax, velocity);


        // Neutral initialising state:
        q_initial = Vector<float>.Build.Dense(this.Articulation.numberJoints);
        q_initial[0] = 0.0f;
        q_initial[1] = -180.0f / 8;
        q_initial[2] = 0;
        q_initial[3] = 3 * 180.0f / 8;
        q_initial[4] = 0;
        q_initial[5] = -100.0f;
        q_initial[6] = 0;

        gripper_goal = gripper_open;

        DynamicInertia = Matrix<float>.Build.Dense(Articulation.numberJoints, Articulation.numberJoints);
        GravityCompForces = Vector<float>.Build.Dense(Articulation.numberJoints);
        CoriolisMatrix = Matrix<float>.Build.Dense(Articulation.numberJoints, Articulation.numberJoints);

        qd_state = Vector<float>.Build.Dense(Articulation.numberJoints);
        qdd_state = Vector<float>.Build.DenseOfVector(qd_state);

        // Reset inertia tensor diagonals to match actual robot data (could also do this for mass)
        foreach (Segment segment in Articulation.segments)
        {
            segment.linkedBody.inertiaTensor.Set(inertiaArrays[segment.jointCount][1], inertiaArrays[segment.jointCount][3], inertiaArrays[segment.jointCount][5]);
        }

        // set up static coefficients:
        CalculateChristoffel();

    }

    public void UpdateJointDynamics()
    {
        for (int i = 0; i < Articulation.numberJoints; i++)
        {
            Segment seg;
            Articulation.chainMap.TryGetValue(i, out seg);
            qd_state[i] = seg.linkedBody.jointVelocity[0];

            // acceleration estimates can be thrown off by high stiffness joints outside the direct control chain (eg
            // additional end effector DoFs). Instead of taking a reading directly from the internal state reporter,
            // can we filter the velocity?
            qdd_state[i] = seg.linkedBody.jointAcceleration[0];
        }
    }

    public List<float> CreateJointTorqueVector(Vector<float> torqueApply)
    {
        List<float> fullJointTorqueList = new List<float>(new float[(int)(this.Articulation.totalDoF)]);

        // add control torques -  can I just sub in a range here?
        // note: this will only add joint torques for the main articulation, gripper controls must be handled separately
        int tCount = 0;
        foreach (float torque in torqueApply) { fullJointTorqueList[tCount] = torque; tCount++; }
        return fullJointTorqueList;
    }

    public Vector<float> SolveInverseKinematics(Matrix4x4 goalPose, Vector<float> q_init, params float[] weights)
    {
        int cycles = 0;
        bool TargetReached = false;
        Vector<float> q_update = Vector<float>.Build.Dense(Articulation.numberJoints);
        Vector<float> q_original = Vector<float>.Build.DenseOfVector(q_update);
        q_init.CopyTo(q_original);

        while (!TargetReached)
        {
            IKSolver.CartToJnt(Articulation, q_init, goalPose, q_original, out TargetReached, weights).CopyTo(q_update);
            // update simulated kinematics
            Articulation.UpdateChainKinematics(q_update);

            // update segment positions, rotations, get end-effector positions
            Articulation.JntToCart();
            q_update.CopyTo(q_init); 
            cycles++;
            if (cycles > maxCycles)
            {
                Debug.Log("Not converging, current state:");
                Debug.Log(q_update);
                break;
            }

        }

        return q_update;
    }

    public bool InsidePandaWorkspace(Matrix4x4 InputPose)
    {
        // Checl lateral extension required:
        Vector2 lateralDistance = new Vector2(InputPose.m03, InputPose.m23) - new Vector2(Articulation.rootPosition.x, Articulation.rootPosition.z);
        if (lateralDistance.magnitude > panda_lateral_radius) { return false; }

        // Identify vertical plane quadrant
        Vector2 yzPosition = new Vector2(InputPose.m13, InputPose.m23) - new Vector2(Articulation.rootPosition.y, Articulation.rootPosition.z);

        // If RHS: check vertical radius required
        // (solve for spiral? we don't have enough precise points)
        if (yzPosition.y > 0)
        {
            if (yzPosition.x < panda_RHymin) { return false; }
            else if (yzPosition.x > panda_RHymax) { return false; }
        }

        // If LHS: Could do an IK check for joint limit violations, until then
        // assume 0->600 is unreachable (this will eliminate certain technically reachable points)
        else if (yzPosition.y < 0)
        {
            if (yzPosition.x < 600) { return false; }
            else if (yzPosition.x < 0)
            {
                // assume (-320, -200) gives the radial limit for lower LHS coordinates.
                if (yzPosition.x < -320) { return false; }
                else if (yzPosition.y < -200) { return false; }
            }
        }

        // if none of these are triggered, we assume the position is theoretically reachable (but this will not always be true)
        // TODO: implement a more precise workspace check mechanism.
        return true;
    }

    public bool InsideVelocityLimits(Matrix4x4 InputPose, float dt)
    {
        // checks whether a given pose (including orientation) is reachable within one fixed time steps,
        // given joint velocity limits. Returns 1 if reachable, 0 if not

        // this is approximate and valid only for small changes in pose (relies on jacobian)
        Vector<float> poseError = IKSolver.CalcPoseError(Articulation.base2EETransform, InputPose);
        // Unity issue: can't access fixedDeltaTime from an external class. Have to send through as dt
        Vector<float> dX = poseError / dt;

        Jacobian cJ = Articulation.BuildJacobian(Articulation.jointState, 1.0f);
        cJ.SVDPseudoInverse();

        Vector<float> dTheta = cJ.inverse * dX;
        if (Mathf.Abs(dTheta.Maximum()) > velocity) { return false; } // could also use a forall check
        else { return true; }

    }

    public void DriveJoints(Vector<float> q_goal)
    {
        // drives joints directly to a position, bounded only by maximum joint force. Velocity bound checking
        // is assumed to occur externally if using this drive method.
        int joint_count = 0;
        foreach (Segment seg in Articulation.segments)
        {

            ArticulationDrive controlDrive = seg.linkedBody.xDrive;
            ArticulationJointType switchType = seg.linkedBody.jointType;

            switch (switchType)
            {
                case ArticulationJointType.PrismaticJoint:
                    {
                        break;
                    }

                case ArticulationJointType.RevoluteJoint:
                    {
                        // Update joint angle target
                        float deltaRotate = q_goal[joint_count] - Mathf.Rad2Deg * seg.linkedBody.jointPosition[0];
                        float state_update = seg.linkedBody.xDrive.target;
                        float currentVelRads = seg.linkedBody.jointVelocity[0];

                        if (Mathf.Abs(deltaRotate) > 0.05)
                        {
                            state_update = q_goal[joint_count];// Mathf.Rad2Deg * seg.linkedBody.jointPosition[0] + deltaRotate;
                        }

                        controlDrive = seg.linkedBody.xDrive;
                        controlDrive.target = state_update;

                        // insert stiffness update here if necessary

                        joint_count++;
                        break;
                    }

                default:
                    {
                        break;
                    }
            }

            seg.linkedBody.xDrive = controlDrive;
        }
    }

    public void DriveJointsIncremental(Vector<float> q_start, Vector<float> q_goal)
    {
        int joint_count = 0;
        foreach (Segment seg in Articulation.segments)
        {

            ArticulationDrive controlDrive = seg.linkedBody.xDrive;
            ArticulationJointType switchType = seg.linkedBody.jointType;

            switch (switchType)
            {
                case ArticulationJointType.PrismaticJoint:
                    {
                        break;
                    }

                case ArticulationJointType.RevoluteJoint:
                    {
                        // Update joint angle target
                        float deltaRotate = q_goal[joint_count] - Mathf.Rad2Deg * seg.linkedBody.jointPosition[0];
                        float state_update = seg.linkedBody.xDrive.target;
                        float currentVelRads = seg.linkedBody.jointVelocity[0];

                        if (Mathf.Abs(deltaRotate) > 0.05)
                        {
                            float deltaState = Mathf.Sign(deltaRotate) * Mathf.Min(velocity * Time.fixedDeltaTime, Mathf.Abs(deltaRotate));
                            state_update = Mathf.Rad2Deg * seg.linkedBody.jointPosition[0] + deltaState;
                        }
                        controlDrive = seg.linkedBody.xDrive;
                        controlDrive.target = state_update;

                        // Update stiffness - for all joints, or some joints?
                        float joint_error = q_goal[joint_count] - Mathf.Rad2Deg * seg.linkedBody.jointPosition[0];
                        float newK = VSAErrorStiffness(seg,  currentVelRads, joint_error);

                        if (seg.VSA_ControlFlag)
                        {
                            controlDrive.stiffness = newK;
                        }

                        joint_count++;
                        break;
                    }

                default:
                    {
                        break;
                    }
            }

            seg.linkedBody.xDrive = controlDrive;
        }

    }

    public void DriveSingleJointIncremental(int jointCount, float q_goal)
    {
        Segment seg;
        Articulation.chainMap.TryGetValue(jointCount, out seg);
        ArticulationDrive controlDrive = seg.linkedBody.xDrive;
        ArticulationJointType switchType = seg.linkedBody.jointType;

        switch (switchType)
        {
            case ArticulationJointType.PrismaticJoint:
                {
                    break;
                }

            case ArticulationJointType.RevoluteJoint:
                {
                    // Update joint angle target
                    float deltaRotate = q_goal - Mathf.Rad2Deg * seg.linkedBody.jointPosition[0];
                    float state_update = seg.linkedBody.xDrive.target;
                    float currentVelRads = seg.linkedBody.jointVelocity[0];

                    if (Mathf.Abs(deltaRotate) > 0.05)
                    {
                        float deltaState = Mathf.Sign(deltaRotate) * Mathf.Min(velocity * Time.fixedDeltaTime, Mathf.Abs(deltaRotate));
                        state_update = Mathf.Rad2Deg * seg.linkedBody.jointPosition[0] + deltaState;
                    }
                    controlDrive = seg.linkedBody.xDrive;
                    controlDrive.target = state_update;

                    // Update stiffness - for all joints, or some joints?
                    float joint_error = q_goal - Mathf.Rad2Deg * seg.linkedBody.jointPosition[0];
                    float newK = VSAErrorStiffness(seg, currentVelRads, joint_error);

                    if (seg.VSA_ControlFlag)
                    {
                        controlDrive.stiffness = newK;
                    }
                    break;
                }

            default:
                {
                    break;
                }
        }

        seg.linkedBody.xDrive = controlDrive;
    }
    

    public void SetStiffness(Segment thisSeg, float kvalue)
    {
        ArticulationDrive controlDrive = thisSeg.linkedBody.xDrive;
        controlDrive.stiffness = kvalue;
        thisSeg.linkedBody.xDrive = controlDrive;
    }


    public void SetDamping(Segment thisSeg, float dvalue)
    {

        ArticulationDrive controlDrive = thisSeg.linkedBody.xDrive;
        controlDrive.damping = dvalue;
        thisSeg.linkedBody.xDrive = controlDrive;
    }

    public float VSAErrorStiffness(Segment thisSeg, float currentVel, float currentError)
    {
        float newK = thisSeg.joint.ks_min + Mathf.Pow(25.0f * currentError * Time.fixedDeltaTime / currentVel, 2.0f);

        if (newK < thisSeg.joint.ks_min) { newK = thisSeg.joint.ks_min; }
        if (newK > thisSeg.joint.ks_max) { newK = thisSeg.joint.ks_max; }

        return newK;
    }

    public void UpdateGripperStates(float manip_target)
    {
        Vector3[] gripLocate = new Vector3[Articulation.Gripper.manipulators.Count];
        int i = 0;

        foreach (ArticulationBody manip in Articulation.Gripper.manipulators)
        {
            ArticulationDrive controlDrive = manip.xDrive;
            controlDrive.target = manip_target;
            manip.xDrive = controlDrive;
            gripLocate[i] = manip.transform.position;
            i++;
        }

        if (i < 3) { GripDistance = (gripLocate[1] - gripLocate[0]).magnitude; }
        // How to get the distance between three grippers? multidimensional? Minimum?

    }

    public void UpdateJointStiffness(Vector<float> jointStiffness)
    {
        foreach (Segment seg in Articulation.segments)
        {
            ArticulationJointType switchType = seg.linkedBody.jointType;
            switch (switchType)
            {
                case ArticulationJointType.PrismaticJoint:
                    {
                        SetStiffness(seg, jointStiffness[seg.jointCount]);
                        break;
                    }

                case ArticulationJointType.RevoluteJoint:
                    {
                        SetStiffness(seg, jointStiffness[seg.jointCount]);
                        break;
                    }

                default: { break; }
            }

        }

    }

    public void UpdateJointDamping(Vector<float> jointDamping)
    {
        foreach (Segment seg in Articulation.segments)
        {
            ArticulationJointType switchType = seg.linkedBody.jointType;
            switch (switchType)
            {
                case ArticulationJointType.PrismaticJoint:
                    {
                        SetDamping(seg, jointDamping[seg.jointCount]);
                        break;
                    }

                case ArticulationJointType.RevoluteJoint:
                    {
                        SetDamping(seg, jointDamping[seg.jointCount]);
                        break;
                    }

                default: { break; }
            }

        }

    }

    // Joint stiffness tuning based on cartesian stiffness goal states

    public Matrix<float> CartToJntStiffness(Vector<float> CartesianStiffness)
    {
        Jacobian cJ = Articulation.BuildJacobian(Articulation.jointState); // Current jacobian
        cJ.SVDPseudoInverse();
        Matrix<float> SCMatrix = Matrix<float>.Build.DenseOfDiagonalVector(CartesianStiffness);
        Matrix<float> SJMatrix = cJ.inverse * SCMatrix * cJ.value;
        return SJMatrix;
    }

    public void DirectForceDrive(float inputTorque, Segment thisSegment)
    {
        // need to make sure that the joint index is correct
        thisSegment.linkedBody.AddRelativeTorque(inputTorque * thisSegment.jointIndex);
    }

    public void ApplyJointForces(Vector<float> inputTorque)
    {
        int jointCount = 0;
        foreach (Segment seg in this.Articulation.segments)
        {
            if (seg.joint.jointType == ArticulationJointType.RevoluteJoint)
            {
                ArticulationDrive jointDriver = seg.linkedBody.xDrive;
        
                seg.linkedBody.xDrive = jointDriver;
                this.DirectForceDrive(inputTorque[jointCount], seg);

                jointCount++;
            }
        }
    }

    // --- Tuned (self balancing) impedence control for no payload
    // (this should let us set spring/damper values more accurately than current setup)
    // need to accurately estimate mass and coriolis forces in cartesian space and apply to
    // joint space
    // Calculate mass matrix and coriolis matrix
    // calculate cartesian control input required
    // calculate torque required
    // Calculate stiffness/damping gains?

    public void CalculateInertiaMatrix()
    {

        Matrix<float> HSum = Matrix<float>.Build.DenseIdentity(Articulation.numberJoints);
        List<Jacobian> localJacobians = new List<Jacobian>();

        foreach (Segment seg in Articulation.segments)
        {
            if (seg.joint.jointType != ArticulationJointType.FixedJoint)
            {
                Jacobian localJac = BuildLocalJacobian(seg.index);

                // store local Jacobian for use by gravity builder
                localJacobians.Add(localJac);

                Matrix<float> CartJac = Matrix<float>.Build.Dense(6, Articulation.numberJoints);
                CartJac.SetRow(0, localJac.value.Row(0));
                CartJac.SetRow(1, localJac.value.Row(1));
                CartJac.SetRow(2, localJac.value.Row(2));

                Matrix<float> RotJac = Matrix<float>.Build.Dense(6, Articulation.numberJoints);
                RotJac.SetRow(3, localJac.value.Row(3));
                RotJac.SetRow(4, localJac.value.Row(4));
                RotJac.SetRow(5, localJac.value.Row(5));

                Matrix<float> localInertia = Matrix<float>.Build.DenseIdentity(6);
                Matrix<float> HLocal = Matrix<float>.Build.DenseIdentity(7);

                // if we have reached the last segment: add eeBody components
                if (seg.jointCount == (Articulation.numberJoints-1))
                {
                    // add cumulative mass from end effector and manipulator
                    float totalLinkMass = seg.linkedBody.mass;
                    totalLinkMass += Articulation.eeBody.mass + Articulation.Gripper.manipMass;

                    // add cumulative inertia for EEBody: 
                    Vector3 offsetVector1 = Articulation.eeBody.worldCenterOfMass - seg.linkedBody.worldCenterOfMass;
                    Matrix<float> localEETensor = Matrix<float>.Build.DenseOfDiagonalArray(new float[] { Articulation.eeBody.inertiaTensor.x, Articulation.eeBody.inertiaTensor.y, Articulation.eeBody.inertiaTensor.z });
                    float offsetDistance1 = Vector3.Dot(offsetVector1, offsetVector1);
                    Vector<float> conversionVector1 = Vector<float>.Build.DenseOfArray(new float[] { offsetVector1.x, offsetVector1.y, offsetVector1.z }); // just to make this cleaner to read
                    Matrix<float> offsetOuterProduct1 = conversionVector1.OuterProduct(conversionVector1);
                    Matrix<float> parallelAxis1 = Articulation.eeBody.mass * (offsetDistance1 * Matrix<float>.Build.DenseIdentity(3) - offsetOuterProduct1);

                    // Add cumulative inertia from manipulator (ignoring rotation around the axis):
                    Vector3 offsetVector2 = Articulation.Gripper.hand.worldCenterOfMass - seg.linkedBody.worldCenterOfMass;
                    Matrix<float> gripperTensor = Matrix<float>.Build.DenseOfDiagonalArray(new float[] { Articulation.Gripper.manipInertia.x, Articulation.Gripper.manipInertia.y, Articulation.Gripper.manipInertia.z });
                    float offsetDistance2 = Vector3.Dot(offsetVector2, offsetVector2);
                    Vector<float> conversionVector2 = Vector<float>.Build.DenseOfArray(new float[] { offsetVector2.x, offsetVector2.y, offsetVector2.z }); // just to make this cleaner to read
                    Matrix<float> offsetOuterProduct2 = conversionVector2.OuterProduct(conversionVector2);
                    Matrix<float> parallelAxis2 = Articulation.Gripper.manipMass * (offsetDistance2 * Matrix<float>.Build.DenseIdentity(3) - offsetOuterProduct2);


                    localInertia[0, 0] = 0;
                    localInertia[1, 1] = 0;
                    localInertia[2, 2] = 0;
                    localInertia[3, 3] = seg.linkedBody.inertiaTensor[0];
                    localInertia[3, 3] += (localEETensor[0, 0] + parallelAxis1[0, 0]) + (gripperTensor[0, 0] + parallelAxis2[0, 0]);
                    localInertia[4, 4] = seg.linkedBody.inertiaTensor[1];
                    localInertia[4, 4] += (localEETensor[1, 1] + parallelAxis1[1, 1]) + (gripperTensor[1, 1] + parallelAxis2[1, 1]);
                    localInertia[5, 5] = seg.linkedBody.inertiaTensor[2];
                    localInertia[5, 5] += (localEETensor[2, 2] + parallelAxis1[2, 2]) + (gripperTensor[2, 2] + parallelAxis2[2, 2]);


                    HLocal = totalLinkMass * (CartJac.Transpose() * CartJac) + RotJac.Transpose() * localInertia * RotJac;
                }

                else
                {
                    localInertia[0, 0] = 0;
                    localInertia[1, 1] = 0;
                    localInertia[2, 2] = 0;
                    localInertia[3, 3] = seg.linkedBody.inertiaTensor[0];
                    localInertia[4, 4] = seg.linkedBody.inertiaTensor[1];
                    localInertia[5, 5] = seg.linkedBody.inertiaTensor[2];

                    HLocal = seg.linkedBody.mass * (CartJac.Transpose() * CartJac) + RotJac.Transpose() * localInertia * RotJac;
                }

                HSum += HLocal;
            }


        }

        HSum.CopyTo(DynamicInertia);

        // Build gravity vector:
        foreach (Segment seg in Articulation.segments)
        {
            if (seg.joint.jointType != ArticulationJointType.FixedJoint)
            {
                GravityCompForces[seg.jointCount] = CalculateGravMassElement(seg.jointCount, localJacobians);
            }
        }
    }

    public float CalculateGravMassElement(int vecIndex, List<Jacobian> jacList)
    {
        float g_init = 0.0f;
        
        foreach (Segment seg in Articulation.segments)
        {

            if (seg.joint.jointType != ArticulationJointType.FixedJoint)
            {
                // Get jacobian of joint J
                Jacobian linkJacobian = jacList[seg.jointCount];

                // get i'th column
                Vector<float> jacVec = linkJacobian.value.Column(vecIndex);

                // add to joint grav compensation
                Vector3 localJac = new Vector3(jacVec[0], jacVec[1], jacVec[2]);
                g_init += seg.linkedBody.mass * Vector3.Dot(Physics.gravity, localJac);

            }
        }

        return -g_init;
    }


    public void CalculateChristoffel()
    {
        // TODO  build a frame class

        Matrix<float>[] linkRotations = new Matrix<float>[Articulation.numberJoints];
        float[,,,] MArray = new float[3, Articulation.numberJoints, Articulation.numberJoints, Articulation.numberJoints]; 
        float[,,,] FArray = new float[3, Articulation.numberJoints, Articulation.numberJoints, Articulation.numberJoints];
        ChrCo = new float[Articulation.numberJoints, Articulation.numberJoints, Articulation.numberJoints];

        foreach (Segment seg in Articulation.segments)
        {
            if (seg.joint.jointType != ArticulationJointType.FixedJoint)
            {
                // use system numerics to do the quaternion to matrix transform (just so we don't have to write this out explicitly)
                System.Numerics.Matrix4x4 tempRotation = System.Numerics.Matrix4x4.CreateFromQuaternion(new System.Numerics.Quaternion(seg.segmentRotation.x, seg.segmentRotation.y, seg.segmentRotation.z, seg.segmentRotation.w));
                float[] tempRotVec = new float[9];
                tempRotVec[0] = tempRotation.M11;
                tempRotVec[1] = tempRotation.M21;
                tempRotVec[2] = tempRotation.M31;
                tempRotVec[3] = tempRotation.M12;
                tempRotVec[4] = tempRotation.M22;
                tempRotVec[5] = tempRotation.M32;
                tempRotVec[6] = tempRotation.M13;
                tempRotVec[7] = tempRotation.M23;
                tempRotVec[8] = tempRotation.M33;

                // recast rotation elements as a mathnet matrix
                linkRotations[seg.jointCount] = Matrix<float>.Build.Dense(3, 3, tempRotVec);
            }
        }

        foreach (Segment seg in Articulation.segments)
        {
            Matrix<float> linkInertia = Matrix<float>.Build.Dense(3, 3);

            if (seg.joint.jointType != ArticulationJointType.FixedJoint)
            {
                // build full inertia matrix for link
                Matrix<float> In_i = Matrix<float>.Build.Dense(3, 3);
                In_i[0, 0] = inertiaArrays[seg.jointCount][0];
                In_i[0, 1] = inertiaArrays[seg.jointCount][1];
                In_i[0, 2] = inertiaArrays[seg.jointCount][2];
                In_i[1, 1] = inertiaArrays[seg.jointCount][3];
                In_i[1, 2] = inertiaArrays[seg.jointCount][4];
                In_i[2, 2] = inertiaArrays[seg.jointCount][5];
                In_i[1, 0] = In_i[0, 1];
                In_i[2, 0] = In_i[0, 2];
                In_i[2, 1] = In_i[1, 2];


                linkInertia = 0.5f * linkRotations[seg.jointCount] * ((In_i[0, 0] + In_i[1, 1] + In_i[2, 2]) * Matrix<float>.Build.DenseIdentity(3) - 2 * In_i * linkRotations[seg.jointCount].Transpose());

                // calculation moment, force components
                Vector<float> pc_i = Vector<float>.Build.DenseOfArray(new float[] { seg.segmentPosition.x, seg.segmentPosition.y, seg.segmentPosition.z });
                pc_i += linkRotations[seg.jointCount] * Vector<float>.Build.DenseOfArray(new float[] { seg.linkedBody.centerOfMass.x, seg.linkedBody.centerOfMass.y, seg.linkedBody.centerOfMass.z });

                // build moment, force arrays by iterating over joints:
                // call joint iterator to update arrays (note: could update Moment and Force arrays in the same loops)
                MomentArrayIterator(seg.jointCount, linkInertia, seg.linkedBody.mass, linkRotations, pc_i, out MArray, out FArray);
            }
        }
        // Reverse recursion to calculate Christoffel elements

        Matrix<float> MacSum1 = Matrix<float>.Build.Dense(Articulation.numberJoints, Articulation.numberJoints);
        Matrix<float> MacSum2 = Matrix<float>.Build.DenseOfMatrix(MacSum1);
        Matrix<float> MacSum3 = Matrix<float>.Build.DenseOfMatrix(MacSum1);
        Matrix<float> FacSum1 = Matrix<float>.Build.DenseOfMatrix(MacSum1);
        Matrix<float> FacSum2 = Matrix<float>.Build.DenseOfMatrix(MacSum1);
        Matrix<float> FacSum3 = Matrix<float>.Build.DenseOfMatrix(MacSum1);


        for (int i = Articulation.numberJoints-1; i > -1; i--)
        {
            Segment currentSeg;
            Articulation.chainMap.TryGetValue(i, out currentSeg);
            Vector3 L = new Vector3();
            Vector<float> pc_ii = linkRotations[i] * Vector<float>.Build.DenseOfArray(new float[] { currentSeg.linkedBody.centerOfMass.x, currentSeg.linkedBody.centerOfMass.y, currentSeg.linkedBody.centerOfMass.z });

            if (i != Articulation.numberJoints-1)
            {
                Segment nextSeg;
                // could also use parent child relationship here
                Articulation.chainMap.TryGetValue(i+1, out nextSeg);
                L = nextSeg.segmentPosition - currentSeg.segmentPosition;
            }

            for (int j = 0; j < Articulation.numberJoints; j++)
            {
                for (int m = j; m < Articulation.numberJoints; m++)
                {
                    MacSum1[m, j] += MArray[0, m, j, i] + L.y * FacSum3[m, j] - L.z * FacSum2[m, j] + pc_ii[1] * FArray[2, m, j, i] - pc_ii[2] * FArray[1, m, j, i];
                    MacSum2[m, j] += MArray[1, m, j, i] - L.x * FacSum3[m, j] + L.z * FacSum1[m, j] - pc_ii[0] * FArray[2, m, j, i] + pc_ii[2] * FArray[1, m, j, i];
                    MacSum3[m, j] += MArray[2, m, j, i] + L.x * FacSum2[m, j] - L.y * FacSum1[m, j] + pc_ii[0] * FArray[2, m, j, i] - pc_ii[1] + FArray[1, m, j, i];

                    FacSum1 += FArray[0, m, j, i];
                    FacSum2 += FArray[1, m, j, i];
                    FacSum3 += FArray[2, m, j, i];

                    ChrCo[m, j, i] = linkRotations[i][0, 2] * MacSum1[m, j] + linkRotations[i][1, 2] * MacSum2[m, j] + linkRotations[i][2, 2] * MacSum3[m, j];
                    ChrCo[j, m, i] = ChrCo[m, j, i];
                }
            }

        }

    }

    public void MomentArrayIterator(int index, Matrix<float> linkInertia, float linkMass, Matrix<float>[] linkRotations, Vector<float> pci, out float[,,,] MArray, out float[,,,] FArray)
    {
        MArray = new float[3, Articulation.numberJoints, Articulation.numberJoints, Articulation.numberJoints];
        FArray = new float[3, Articulation.numberJoints, Articulation.numberJoints, Articulation.numberJoints];

        Vector<float> F_tij = Vector<float>.Build.Dense(3);
        Vector<float> Pcim = Vector<float>.Build.Dense(3);

        for (int j = 0; j< index+1; j++)
        {
            Segment currentSeg;
            Articulation.chainMap.TryGetValue(j, out currentSeg);
            Vector<float> Tj_cart = Vector<float>.Build.DenseOfArray(new float[] { currentSeg.segmentPosition.x, currentSeg.segmentPosition.y, currentSeg.segmentPosition.z });

            // moment array calcs
            float t1 = linkInertia.Row(0).DotProduct(linkRotations[j].Column(2));
            float t2 = linkInertia.Row(1).DotProduct(linkRotations[j].Column(2));
            float t3 = linkInertia.Row(2).DotProduct(linkRotations[j].Column(2));

            // force array calcs
            F_tij = linkMass * linkRotations[j].Column(2);

            for (int m = j; m < index+1; m++)
            {
                // moment array calcs
                MArray[0, m, j, index] = t2 * linkRotations[m][2,2] - t3 * linkRotations[m][1, 2];
                MArray[1, m, j, index] = -t1 * linkRotations[m][2, 2] + t3 * linkRotations[m][0, 2];
                MArray[2, m, j, index] = t1 * linkRotations[m][1, 2] - t2 * linkRotations[m][0, 2];

                // force array calcs
                Pcim = pci - Tj_cart;
                float tf1 = linkRotations[m][1, 2] * Pcim[2] - linkRotations[m][2, 2] * Pcim[1];
                float tf2 = -linkRotations[m][0, 2] * Pcim[2] + linkRotations[m][2, 2] * Pcim[0];
                float tf3 = linkRotations[m][0, 2] * Pcim[1] - linkRotations[m][1, 2] * Pcim[0];

                FArray[0, m, j, index] = F_tij[1] * tf3 - F_tij[2] * tf2;
                FArray[1, m, j, index] = -F_tij[0] * tf3 + F_tij[2] * tf1;
                FArray[2, m, j, index] = F_tij[0] * tf2 - F_tij[1] * tf1;
            }
        }

    }



    public void UpdateCoriolis()
    {
        // C_ij = Sum(k=0-->n) (Chris_ijk*dtheta_k)
        for (int i = 0; i < Articulation.numberJoints; i++)
        {
            for (int j=0; j < Articulation.numberJoints; j++)
            {
                // there must be a more elegant way to build this slice
                CoriolisMatrix[i, j] = (Vector<float>.Build.Dense(new float[] {
                    ChrCo[i, j,0], ChrCo[i,j,1], ChrCo[i,j,2], ChrCo[i,j,3], ChrCo[i,j,4], ChrCo[i,j,5], ChrCo[i,j,6] })).DotProduct(qd_state);
            }
        }

    }

    public Jacobian BuildLocalJacobian(int segmentIndex)
    {
        // predeclare temporary variables
        Vector<float>[] JacArray = new Vector<float>[Articulation.numberJoints];

        // fill with zeros
        for (int jCount = 0; jCount < Articulation.numberJoints; jCount++)
        {
            JacArray[jCount] = Vector<float>.Build.Dense(Articulation.numberJoints);
        }

        Vector<float> jacItem = Vector<float>.Build.Dense(6);

        // local segment position
        Vector3 pEE = Articulation.segmentPositions[segmentIndex];
        int jj = 0;

        foreach (Segment seg in Articulation.segments)
        {
            if (seg.index > segmentIndex){ break; }

            if (seg.joint.jointType != ArticulationJointType.FixedJoint)
            {
                // Generate full transformation matrix
                Matrix4x4 segmentTransform = Matrix4x4.TRS(Articulation.segmentPositions[seg.index], Articulation.segmentRotations[seg.index], new Vector3(1, 1, 1));

                // Linear velocity component
                Vector3 cartDelta = pEE - Articulation.segmentPositions[seg.index];

                // IMPORT PROBLEM! some inconsistency in anchor rotations on import from URDF. Tune Jacobian to rotation axis
                // fix this
                Vector3 segAxis;

                if (seg.jointCount == 0) { segAxis = -new Vector3(segmentTransform.m02, segmentTransform.m12, segmentTransform.m22); }
                else if (seg.jointCount == 1) { segAxis = new Vector3(segmentTransform.m02, segmentTransform.m12, segmentTransform.m22); }
                else { segAxis = -new Vector3(segmentTransform.m01, segmentTransform.m11, segmentTransform.m21); }

                Vector3 Jv = Vector3.Cross(cartDelta, segAxis);

                jacItem[0] = Jv.x;
                jacItem[1] = Jv.y;
                jacItem[2] = Jv.z;
                jacItem[3] = segAxis.x;
                jacItem[4] = segAxis.y;
                jacItem[5] = segAxis.z;

                // Condition vector elements
                for (int kk = 0; kk < jacItem.Count; kk++)
                {
                    if (Mathf.Abs(jacItem[kk]) < Articulation.jac_condition_eps) { jacItem[kk] = 0.0f; }
                }

                JacArray[jj] = Vector<float>.Build.Dense(6);
                jacItem.CopyTo(JacArray[jj]);
                jj++;
            }
        }
        return new Jacobian(JacArray);
    }

    public float InertialMassAboveJoint(Segment thisSegment)
    {
        // returns cumulative mass of all child objects from given segment
        ArticulationBody[] childBodies = thisSegment.linkedBody.GetComponentsInChildren<ArticulationBody>();
        Vector<float> childMasses = Vector<float>.Build.Dense(childBodies.Length);
        float cumulativeMass = 0.0f;
        foreach (ArticulationBody body in childBodies)
        {
            cumulativeMass += body.mass;
        }
        return cumulativeMass;
    }

    public Vector3 CumulativeCOMAboveJoint(Segment thisSegment)
    {
        ArticulationBody[] childBodies = thisSegment.linkedBody.GetComponentsInChildren<ArticulationBody>();
        Vector3 cumulativeCoM = new Vector3(0, 0, 0);
        float cumulativeMass = 0.0f;
        foreach (ArticulationBody body in childBodies)
        {
            // need to get locally referenced centre of mass
            Vector3 weightedCOM = (body.worldCenterOfMass - thisSegment.linkedBody.worldCenterOfMass) * body.mass;
            cumulativeCoM += weightedCOM;
            cumulativeMass += body.mass;
        }
        return cumulativeCoM / cumulativeMass;

    }

    public Matrix<float> ResidualSkewMatrix(Matrix<float> inputMatrix)
    {
        // create new skew-symmetric matrix from a symmetric input matrix by setting diagonals to zero
        float[] resetDiagonals = new float[inputMatrix.RowCount];
        Matrix<float> ResidualMatrix = Matrix<float>.Build.DenseOfMatrix(inputMatrix);
        ResidualMatrix.SetDiagonal(resetDiagonals); ; // making sure we do a deep copy here
        return ResidualMatrix;
    }

}

