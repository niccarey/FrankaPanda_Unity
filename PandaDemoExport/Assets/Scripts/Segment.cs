using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

// Segment: extend Articulation Body

public class Segment
{

    // Note: Segment used to inherit from Articulation Body but this led to redundant memory chunks
    // Now just points to the linked articulation body in structure.
    // Assume segment geometry is fixed.

    // Need to add Pose and Twist outputs eventually to speed up local geometry calculations ?

    public ArticulationBody linkedBody;
    public ArticulationBody Parent;      // when we set Parent we need to open the parent and recursively set Child.
    public ArticulationBody Child;

    public Vector3 localPosition;        // localPosition/rotation includes any joint rotatoin.
    public Quaternion localRotation;

    public Vector3 segmentPosition;
    public Quaternion segmentRotation;   // segment position and rotation relative to parent axis, with no joint information

    public float localJointState;
    public bool VSA_ControlFlag = false; 


    // use linkedBody.transform to access the actual current state of the AB
    // local info is instantiated using elements from linkedBody.transform but is not updated as the body moves
    // we use this for forward state projectsions (eg solving IK problem).

    public int index;                   // keep track of segment index
    public int jointCount;              // keep track of linked joint index
    public Vector3 jointIndex;          // set axis of rotation relative to parent
    public Vector3 jointWorldIndex;     // set axis of rotation relative to world frame

    public IKJoint joint = null;        // need to have something we can point to as the joint connected to the segment


    public Segment(ArticulationBody newBody, int chain_link)
    {
        // Setting fixed properties

        // probably need error handling here?
        this.linkedBody = newBody;

        localPosition = newBody.transform.localPosition;
        localRotation = newBody.transform.localRotation;

        segmentPosition = newBody.transform.localPosition; // assume we start at joint = 0 on initialisation
        segmentRotation = newBody.transform.localRotation;

        this.index = chain_link;

        // go back and add child to parent
        if (newBody.isRoot)
        {
            Debug.Log("Setting root component");
            this.Parent = null;
            // add a fixed joint to accommodate IK calculations
        }
        else
        {
            this.Parent = newBody.GetComponentInParent<ArticulationBody>() as ArticulationBody;
        }
        this.joint = AddJoint(newBody);

    }

    public void AddChild(Segment newSegment)
    {
        this.Child = newSegment.linkedBody;
    }

    public IKJoint AddJoint(ArticulationBody newBody)
    {
        if (this.joint != null)
        {
            Debug.Log("Warning: segment already has joint");
        }
        else
        {
            this.joint = new IKJoint(newBody);
        }

        // TODO change this to a switch statement

        if (this.joint.jointType == ArticulationJointType.PrismaticJoint)
        {
            // Get local joint axis from anchor rotation (note: drive is always around or along link X axis)
            newBody.anchorRotation.ToAngleAxis(out float anchorAngle, out Vector3 anchorAxis);

            // check wrapping
            if (anchorAngle > 180) { anchorAngle = 180.0f - anchorAngle; }
            if (anchorAngle < -180) { anchorAngle = -180.0f - anchorAngle; }

            // this is a hack required by URDF->Unity import issues
            // now out of date, need to fix this
            if (Mathf.Abs(anchorAxis[1]) > 0) { this.jointIndex = -Mathf.Sign(anchorAngle) * new Vector3(0, 0, 1); }
            else if (Mathf.Abs(anchorAxis[2]) > 0) { this.jointIndex = Mathf.Sign(anchorAngle) * new Vector3(0, 1, 0); }
            else { this.jointIndex = Mathf.Sign(anchorAngle) * new Vector3(1, 0, 0); }

            this.localJointState = newBody.jointPosition[0];
        }
        else if (this.joint.jointType == ArticulationJointType.RevoluteJoint) // revolute
        {
            // Get local joint axis from anchor rotation (note: drive is always around or along link X axis)
            /*newBody.anchorRotation.ToAngleAxis(out float anchorAngle, out Vector3 anchorAxis);

            // check wrapping
            if (anchorAngle > 180) { anchorAngle = 180.0f - anchorAngle; }
            if (anchorAngle < -180) { anchorAngle = -180.0f - anchorAngle; }*/

            // should joint axes be in local or parent frame? Previously: had parent, maybe child is more useful?

            // Anchor rotations have now been updated, this mapping is no longer accurate
            
            if (this.index == 1) { this.jointIndex = new Vector3(0, 0, 1); }
            else if (this.index == 2) { this.jointIndex = new Vector3(0, 0, -1); }
            else if (this.index == 3) { this.jointIndex = new Vector3(0, 1, 0); }
            else if (this.index == 4) { this.jointIndex = new Vector3(0, 1, 0); }
            else if (this.index == 5) { this.jointIndex = new Vector3(0, 1, 0); }
            else if (this.index == 6) { this.jointIndex = new Vector3(0, 1, 0); }
            else if (this.index == 7) { this.jointIndex = new Vector3(0, 1, 0); }
            else { this.jointIndex = new Vector3(0, 1, 0); }

            newBody.transform.localRotation.ToAngleAxis(out float localAngle, out Vector3 localAxis);
            this.localRotation = Quaternion.AngleAxis(localAngle, jointIndex);

            /*if (Mathf.Abs(anchorAxis[1]) > 0) { this.jointIndex = -Mathf.Sign(anchorAngle) * new Vector3(0, 0, 1); }
            else if (Mathf.Abs(anchorAxis[2]) > 0) { this.jointIndex = Mathf.Sign(anchorAngle) * new Vector3(0, 1, 0); }
            else { this.jointIndex = Mathf.Sign(anchorAngle) * new Vector3(1, 0, 0); }*/

            this.localJointState = newBody.jointPosition[0];

        }
        else if (this.joint.jointType == ArticulationJointType.SphericalJoint) //spherical
        {
            this.jointIndex = new Vector3(1, 1, 1);

        }
        else // fixed or null
        {
            this.jointIndex = new Vector3(0, 0, 0);
            this.localJointState = 0.0f;

        }

        return this.joint;
    }

    public Vector<float> JointTwist(float joint_velocity)
    {
        // returns full 6DoF twist based on input joint velocity in local frame
        // TODO: change this to use local parent axis. Currently not in use, low priority.

        Vector<float> ttwist = Vector<float>.Build.Dense(6);
        ttwist.Clear();

        switch (joint.jointType)
        {
            case ArticulationJointType.RevoluteJoint:
                {
                    ttwist[0] = 0.0f;
                    ttwist[1] = 0.0f;
                    ttwist[2] = 0.0f;
                    ttwist[3] = joint_velocity;
                    ttwist[4] = 0.0f;
                    ttwist[5] = 0.0f;
                    return ttwist;
                }

            case ArticulationJointType.PrismaticJoint:
                {
                    ttwist[0] = joint_velocity;
                    ttwist[1] = 0.0f;
                    ttwist[2] = 0.0f;
                    ttwist[3] = 0.0f;
                    ttwist[4] = 0.0f;
                    ttwist[5] = 0.0f;
                    return ttwist;
                }

            case ArticulationJointType.SphericalJoint:
                {
                    
                    ttwist[0] = 0.0f;
                    ttwist[1] = 0.0f;
                    ttwist[2] = 0.0f;
                    ttwist[3] = 0.0f;
                    ttwist[4] = 0.0f;
                    ttwist[5] = 0.0f;
                    return ttwist;
                }

            case ArticulationJointType.FixedJoint:
                {
                    // this may not be the best way of handling a fixed joint, can
                    // also put a handler in the calling function
                    ttwist[0] = 0.0f;
                    ttwist[1] = 0.0f;
                    ttwist[2] = 0.0f;
                    ttwist[3] = 0.0f;
                    ttwist[4] = 0.0f;
                    ttwist[5] = 0.0f;
                    return ttwist;
                }

            default:
                { return ttwist; }
        }

    }


    public Segment getSegment()
    {
        return this;
    }

    public IKJoint getJoint()
    {
        return joint;
    }

}


