using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// joint: create from Articulation Body

public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };

public class IKJoint
{

    public ArticulationBody jointBody;
    public ArticulationJointType jointType;
    public Vector2 jointLimits;
    public float ks_max = 20000.0f; // max joint stiffness
    public float ks_min;
    // To update joints given a fixed time step 
    public RotationDirection rotationState = RotationDirection.None;
    public float speed = 300.0f;

    public IKJoint(ArticulationBody newBody)
    {
        // extend joint to cover root situation
        jointBody = newBody;
        // on initialisation, set minimum joint stiffness:
        ks_min = jointBody.xDrive.stiffness;
        if (newBody.isRoot) { this.jointType = ArticulationJointType.FixedJoint; }
        else
        {
            this.jointType = newBody.jointType;
            this.jointLimits = new Vector2(0.0f, 0.0f);
        }
    }

    
    public void RotateJoint(float deltaRotate)
    {
        rotationState = GetRotationDirection(deltaRotate);

        if (rotationState != RotationDirection.None)
        {
            float rotationChange = (float)rotationState * speed * Time.fixedDeltaTime;
            float rotationGoal = CurrentPrimaryAxisRotation() + rotationChange;
            RotateTo(rotationGoal);
        }
    }

    float CurrentPrimaryAxisRotation()
    {
        float currentRotationRads = jointBody.jointPosition[0];
        float currentRotation = Mathf.Rad2Deg * currentRotationRads;
        return currentRotation;
    }

    void RotateTo(float primaryAxisRotation)
    {
        var drive = jointBody.xDrive;
        drive.target = primaryAxisRotation;
        jointBody.xDrive = drive;
    }

    static RotationDirection GetRotationDirection(float inputVal)
    {
        if (inputVal > 0.01)
        {
            return RotationDirection.Positive;
        }
        else if (inputVal < -0.01)
        {
            return RotationDirection.Negative;
        }
        else
        {
            return RotationDirection.None;
        }
    }
}

