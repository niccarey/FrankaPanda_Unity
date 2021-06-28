using System.Collections;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class CollisionDebugger : MonoBehaviour
{
    void OnCollisionEnter(Collision collision)
    {
        Debug.Log("collision detected");
        Debug.Log(collision.gameObject.name);
    }
}
  