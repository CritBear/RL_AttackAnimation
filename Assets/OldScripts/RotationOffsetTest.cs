using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotationOffsetTest : MonoBehaviour
{
    public Transform target;

    void OnDrawGizmos()
    {
        if(target != null) {
            target.rotation = Quaternion.LookRotation(Vector3.forward, Vector3.up);
        }
    }
}
