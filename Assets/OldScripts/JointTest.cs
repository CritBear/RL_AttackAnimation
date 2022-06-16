using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointTest : MonoBehaviour
{
    private ArticulationBody SelectedJoint;
    
    public enum AxisSelection {
        Axis_X,
        Axis_Y,
        Axis_Z
    };

    public AxisSelection ImpulseDirection = AxisSelection.Axis_X;
    public float ImpulsePower = 100.0f;

    public float delay = 2f;
    private float elapsed = 0;

    void Start() {
        SelectedJoint = gameObject.GetComponent<ArticulationBody>();
    }

    void Update() {
        elapsed += Time.deltaTime;
        if(elapsed > delay) {
            elapsed = 0;
            if(ImpulseDirection == AxisSelection.Axis_X) {
                SelectedJoint.AddRelativeTorque(Vector3.right * ImpulsePower);
            } else if(ImpulseDirection == AxisSelection.Axis_Y) {
                SelectedJoint.AddRelativeTorque(Vector3.up * ImpulsePower);
            } else {
                SelectedJoint.AddRelativeTorque(Vector3.forward * ImpulsePower);
            }
        }
    }
}