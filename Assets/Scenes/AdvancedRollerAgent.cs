using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Random = UnityEngine.Random;

public class AdvancedRollerAgent : Agent
{
    private Rigidbody m_rBody;
    public Rigidbody boxRigidBody;
    public Transform Target;
    public float forceMultiplier = 1f;
    
    void Start()
    {
        m_rBody = gameObject.GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        m_rBody.angularVelocity = Vector3.zero;
        m_rBody.velocity = Vector3.zero;
        m_rBody.ResetInertiaTensor();
        transform.localPosition = new Vector3(Random.Range(-4.5f, 4.5f), transform.localPosition.y, Random.Range(-4.5f, 4.5f));
        
        boxRigidBody.angularVelocity = Vector3.zero;
        boxRigidBody.velocity = Vector3.zero;
        Vector3 randomVector;
        do
        {
            randomVector = new Vector3(Random.Range(-3f, 3f), 0.5f, Random.Range(-3f, 3f));
        } while (Vector3.Distance(randomVector, transform.localPosition) < 1.1f);
        boxRigidBody.transform.localPosition = randomVector;
        boxRigidBody.transform.localRotation = Quaternion.Euler(0f, Random.value * 360f, 0f);

        do
        {
            randomVector = new Vector3(Random.Range(-4.5f, 4.5f), Target.localPosition.y, Random.Range(-4.5f, 4.5f));
        } while (Vector3.Distance(randomVector, boxRigidBody.transform.localPosition) < 1.5f);
        Target.localPosition = randomVector;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(boxRigidBody.transform.localPosition);
        sensor.AddObservation(boxRigidBody.rotation.eulerAngles.y);
        sensor.AddObservation(Target.localPosition);

        sensor.AddObservation(m_rBody.velocity.x);
        sensor.AddObservation(m_rBody.velocity.z);
        sensor.AddObservation(m_rBody.angularVelocity.x);
        sensor.AddObservation(m_rBody.angularVelocity.y);
        sensor.AddObservation(m_rBody.angularVelocity.z);
        sensor.AddObservation(boxRigidBody.velocity.x);
        sensor.AddObservation(boxRigidBody.velocity.z);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        Vector3 controlSignal = Vector3.zero;
        controlSignal.x = actions.ContinuousActions[0];
        controlSignal.y = actions.ContinuousActions[1];
        controlSignal.z = actions.ContinuousActions[2];
        controlSignal *= forceMultiplier;
        m_rBody.AddTorque(controlSignal, ForceMode.Impulse);
    }

    private void FixedUpdate()
    {
        float distanceBoxToTarget = Vector3.Distance(boxRigidBody.transform.localPosition, Target.localPosition);
        float a = 10f;
        float b = -0.3f;
        AddReward(a * Mathf.Exp(b * distanceBoxToTarget * distanceBoxToTarget));

        a = 1f;
        b = -0.5f;
        float distanceToTarget = Vector3.Distance(boxRigidBody.transform.localPosition, transform.localPosition);
        AddReward(a * Mathf.Exp(b * distanceToTarget * distanceToTarget));

        if (transform.localPosition.x > 5f || transform.localPosition.x < -5f ||
            transform.localPosition.z > 5f || transform.localPosition.z < -5f ||
            boxRigidBody.transform.localPosition.x > 4.5f || boxRigidBody.transform.localPosition.x < -4.5f ||
            boxRigidBody.transform.localPosition.z > 4.5f || boxRigidBody.transform.localPosition.z < -4.5f)
        {
            AddReward(-1);
            EndEpisode();
        }

        if (distanceBoxToTarget <= 1f)
        {
            AddReward(100f);
            EndEpisode();
        }
    }
}
