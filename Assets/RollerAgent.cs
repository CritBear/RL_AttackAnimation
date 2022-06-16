using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class RollerAgent : Agent
{
    private Rigidbody rBody;
    public Transform Target;
    public float forceMultiplier = 10;

    void Start()
    {
        rBody = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        // print("Reset: " + ResetCount.ToString());
        // ResetCount++;
        
        if(this.transform.localPosition.y < 0) {
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity = Vector3.zero;
            this.transform.localPosition = new Vector3(0, 0.5f, 0);
        }

        Target.localPosition = new Vector3(Random.value * 8 - 4, 0.5f, Random.value * 8 - 4);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(Target.localPosition);
        sensor.AddObservation(this.transform.localPosition);

        sensor.AddObservation(rBody.velocity.x);
        sensor.AddObservation(rBody.velocity.z);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // print("Action 0 : " + actions.ContinuousActions[0].ToString("F5"));
        // print("Action 1 : " + actions.ContinuousActions[1].ToString("F5"));
        // SetReward(-0.01f);
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actions.ContinuousActions[0];
        controlSignal.z = actions.ContinuousActions[1];
        rBody.AddForce(controlSignal * forceMultiplier);

        float distanceToTarget = Vector3.Distance(this.transform.localPosition, Target.localPosition);

        
        
        // if(distanceToTarget < 1.42f) {
        //     SetReward(1.0f);
        //     //EndEpisode();
        //     //base.NotifyAgentDone(DoneReason.DoneCalled);
        //     Target.localPosition = new Vector3(Random.value * 8 - 4, 0.5f, Random.value * 8 - 4);
        // } else if(this.transform.localPosition.y < 0) {
        //     SetReward(-10.0f);
        //     base.NotifyAgentDone(DoneReason.DoneCalled);
        // }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}
