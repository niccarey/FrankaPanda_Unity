using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

// tool class for articulated linkages
// takes the end effector of a chain of articulated bodies and creates an easy
// reference class for the tool centroid and any actuated children of the body (eg. grippers, pincers)
// TODO: make a subfunction so any changes we make in default constructor are reflected in the overloads.

public class GripperTool
{
    public Vector3 toolVector;
    public Quaternion toolOrientation;
    public List<ArticulationBody> manipulators = new List<ArticulationBody>();
    public Vector3 padding = new Vector3(0.0f, 0.0f, 0.0f);

    public float manipMass; // total manipulator mass
    public Vector3 manipInertia; // inertia of manipulator

    public ArticulationBody hand;

    public GripperTool(ArticulationBody eeBody)
    {
        // Vector between eeBody joint anchor and tool centroid:
        // need to add eeBody (and any parents between eeBody and children)
        Debug.Log(eeBody.transform.position);
        Debug.Log(eeBody.transform.localPosition);
        // Get children
        Component[] childrenBodies = eeBody.GetComponentsInChildren<ArticulationBody>(); // this is recursive, childrenBodies includes itself!

        // look for prismatic or revolute joints
        int tool_i = 0;
        manipMass = 0;

        Debug.Log("Generating gripper data");

        foreach (ArticulationBody cBody in childrenBodies)
        {
            // note that this will include the end-effector itself
            if (cBody.jointType == ArticulationJointType.PrismaticJoint)
            {
                // create a pincer, add it to manipulator control list
                manipulators.Add(cBody);
            }

            // else if (eeBody.jointType == ArticulationJointType.RevoluteJoint) { // create a finger linkage }

            // assume manipulator has hand in the name (this is a hack)
            // need to add child bodies as well
            if (cBody.name.Contains("hand"))
            {
                Debug.Log("Found Hand");
                hand = cBody;
                manipMass = cBody.mass;
                manipInertia = cBody.inertiaTensor;
            }
            tool_i++; // unclear if I need this tbh
        }

        // note: offset between joint of manipulator and grasp point does not come for free,
        // if we want to use it we will have to add it in either the URDF or manually

        // local orientation of eeBody joint anchor
        toolOrientation = eeBody.parentAnchorRotation;

        // Should also adjust for eeBody rotation (ideally is just around x axis so should make no difference but just in case)

        // tool vector should be a single axis translation along the x axis of the last link to the manipulator position.
        Vector3 localGripperTranslation = eeBody.transform.localPosition + hand.transform.localPosition+ manipulators[0].transform.localPosition + padding;
        toolVector = new Vector3(0, 0, localGripperTranslation.y);
        // 0.16m looks about right.
    }

    // can't initialise with a default value so just overload it
    public GripperTool(ArticulationBody eeBody, Vector3 padding)
    {
        // Vector between eeBody joint anchor and tool centroid:

        // Get children
        Component[] childrenBodies = eeBody.GetComponentsInChildren<ArticulationBody>(); // this is recursive, childrenBodies includes itself!

        Debug.Log("Generating gripper data");

        // look for prismatic or revolute joints
        int tool_i = 0;

        foreach (ArticulationBody cBody in childrenBodies)
        {
            // note that this will include the end-effector itself
            if (cBody.jointType == ArticulationJointType.PrismaticJoint)
            {
                // create a pincer, add it to manipulator control list
                // Collider manipCollider = cBody.GetComponent<Collider>(); (doesn't change sinking behaviour)
                // manipCollider.contactOffset = 0.001f;
                manipulators.Add(cBody);                
            }
            if (cBody.name.Contains("hand"))
            {
                Debug.Log("Found Hand");
                hand = cBody;
                manipMass = cBody.mass;
                manipInertia = cBody.inertiaTensor;
            }

            // else if (eeBody.jointType == ArticulationJointType.RevoluteJoint) { // create a finger linkage }

            tool_i++;
        }

        // note: offset between joint of manipulator and grasp point does not come for free,
        // if we want to use it we will have to add it in either the URDF or manually

        // local orientation of eeBody joint anchor
        toolOrientation = eeBody.parentAnchorRotation;

        // Should also adjust for eeBody rotation (ideally is just around x axis so should make no difference but just in case)
        toolVector = eeBody.transform.localPosition + manipulators[0].transform.localPosition + padding;

    }

}
