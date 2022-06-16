using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
using Random = UnityEngine.Random;
using UnityEditor;
using UnityEngine.UI;

public class AttackAgent : Agent
{
    [Header("Target To Hit Towards")] public Transform target; //Target the agent will walk towards during training.

    [Header("Body Parts")] public Transform hips;
    public Transform chest;
    public Transform spine;
    public Transform head;
    public Transform thighL;
    public Transform shinL;
    public Transform footL;
    public Transform thighR;
    public Transform shinR;
    public Transform footR;
    public Transform armL;
    public Transform forearmL;
    public Transform handL;
    public Transform armR;
    public Transform forearmR;
    public Transform handR;

    //This will be used as a stabilized model space reference point for observations
    //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
    OrientationCubeController m_OrientationCube;
    
    JointDriveController m_JdController;
    EnvironmentParameters m_ResetParams;

    Vector3 RootStartPos;
    Quaternion RootStartRot;
    
    private int standTime;

    public Transform weaponPivot;

    private Vector3 prevWeaponPivotPos;
    [HideInInspector] public float weaponAimReward;
    
    [Header("UI")]
    public Slider aimSlider;

    public Text aimRewardText;

    public override void Initialize()
    {
        m_OrientationCube = GetComponentInChildren<OrientationCubeController>();

        RootStartPos = hips.position;
        RootStartRot = hips.rotation;

        //Setup each body part
        m_JdController = GetComponent<JointDriveController>();
        m_JdController.SetupBodyPart(hips);
        m_JdController.SetupBodyPart(chest);
        m_JdController.SetupBodyPart(spine);
        m_JdController.SetupBodyPart(head);
        m_JdController.SetupBodyPart(thighL);
        m_JdController.SetupBodyPart(shinL);
        m_JdController.SetupBodyPart(footL);
        m_JdController.SetupBodyPart(thighR);
        m_JdController.SetupBodyPart(shinR);
        m_JdController.SetupBodyPart(footR);
        m_JdController.SetupBodyPart(armL);
        m_JdController.SetupBodyPart(forearmL);
        m_JdController.SetupBodyPart(handL);
        m_JdController.SetupBodyPart(armR);
        m_JdController.SetupBodyPart(forearmR);
        m_JdController.SetupBodyPart(handR);

        m_ResetParams = Academy.Instance.EnvironmentParameters;

        SetResetParameters();
    }

    public override void OnEpisodeBegin()
    {
        standTime = 0;
        //Reset all of the body parts
        m_JdController.bodyPartsDict[hips].joint.TeleportRoot(RootStartPos, RootStartRot);
        m_JdController.bodyPartsDict[hips].joint.velocity = Vector3.zero;
        m_JdController.bodyPartsDict[hips].joint.angularVelocity = Vector3.zero;
        m_JdController.bodyPartsDict[hips].joint.ResetInertiaTensor();

        foreach (var bodyPart in m_JdController.bodyPartsDict.Values)
        {
            bodyPart.Reset(bodyPart);
        }

        //Random start rotation to help generalize
        //hips.rotation = Quaternion.Euler(0, Random.Range(0.0f, 360.0f), 0);

        UpdateOrientationObjects();

        SetResetParameters();
        target.GetComponent<TargetController>().MoveTargetToRandomPosition();

        prevWeaponPivotPos = weaponPivot.position;
    }

    public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
    {
        //GROUND CHECK
        sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground
        
        sensor.AddObservation(hips.InverseTransformPoint(bp.joint.transform.position));
        sensor.AddObservation(hips.InverseTransformPoint(bp.joint.velocity));
        sensor.AddObservation(hips.InverseTransformPoint(bp.joint.angularVelocity));
        
        if (bp.joint.transform != hips && bp.joint.transform != handL && bp.joint.transform != handR)
        {
            for (int dofIdx = 0; dofIdx < bp.joint.dofCount; dofIdx++)
            {
                sensor.AddObservation(bp.joint.jointPosition[dofIdx]);
                sensor.AddObservation(bp.joint.jointVelocity[dofIdx]);
                sensor.AddObservation(bp.joint.jointAcceleration[dofIdx]);
                sensor.AddObservation(bp.joint.jointForce[dofIdx]);
            }
            
            //sensor.AddObservation(bp.joint.transform.localRotation);
            //sensor.AddObservation(bp.currentStrength / m_JdController.jointForceLimit);
        }
        
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        var cubeForward = m_OrientationCube.transform.forward;

        sensor.AddObservation(hips.InverseTransformDirection(Physics.gravity));
        //rotation deltas
        sensor.AddObservation(Quaternion.FromToRotation(hips.forward, cubeForward));
        //sensor.AddObservation(Quaternion.FromToRotation(head.forward, cubeForward));

        //Position of target position relative to cube
        sensor.AddObservation(hips.InverseTransformPoint(target.transform.position));
        sensor.AddObservation(hips.InverseTransformPoint(weaponPivot.position));
        
        sensor.AddObservation((weaponPivot.position - prevWeaponPivotPos).magnitude);
        sensor.AddObservation(Vector3.Distance(weaponPivot.position, target.transform.position));

        foreach (var bodyPart in m_JdController.bodyPartsList)
        {
            CollectObservationBodyPart(bodyPart, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var bpDict = m_JdController.bodyPartsDict;
        var i = -1;

        var continuousActions = actionBuffers.ContinuousActions;
        bpDict[chest].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[spine].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        bpDict[thighL].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[thighR].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[shinL].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[shinR].SetJointTargetRotation(continuousActions[++i], 0, 0);
        bpDict[footR].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);
        bpDict[footL].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], continuousActions[++i]);

        bpDict[armL].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[armR].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);
        bpDict[forearmL].SetJointTargetRotation(continuousActions[++i], 0, continuousActions[++i]);
        bpDict[forearmR].SetJointTargetRotation(continuousActions[++i], 0, continuousActions[++i]);
        bpDict[head].SetJointTargetRotation(continuousActions[++i], continuousActions[++i], 0);

        //update joint strength settings
        bpDict[chest].SetJointStrength(continuousActions[++i]);
        bpDict[spine].SetJointStrength(continuousActions[++i]);
        bpDict[head].SetJointStrength(continuousActions[++i]);
        bpDict[thighL].SetJointStrength(continuousActions[++i]);
        bpDict[shinL].SetJointStrength(continuousActions[++i]);
        bpDict[footL].SetJointStrength(continuousActions[++i]);
        bpDict[thighR].SetJointStrength(continuousActions[++i]);
        bpDict[shinR].SetJointStrength(continuousActions[++i]);
        bpDict[footR].SetJointStrength(continuousActions[++i]);
        bpDict[armL].SetJointStrength(continuousActions[++i]);
        bpDict[forearmL].SetJointStrength(continuousActions[++i]);
        bpDict[armR].SetJointStrength(continuousActions[++i]);
        bpDict[forearmR].SetJointStrength(continuousActions[++i]);
    }

    void UpdateOrientationObjects()
    {
        m_OrientationCube.UpdateOrientation(hips, target);

    }

    void FixedUpdate()
    {
        UpdateOrientationObjects();

        if (Vector3.Distance(hips.position, RootStartPos) > 5)
        {
            Debug.Log("Explode : " + gameObject.name);
            EndEpisode();
        }

        var cubeForward = m_OrientationCube.transform.forward;

        // b. Rotation alignment with target direction.
        //This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
        /*var lookAtTargetReward = (Vector3.Dot(cubeForward, head.forward) + 1) * .5F;

        //Check for NaNs
        if (float.IsNaN(lookAtTargetReward))
        {
            throw new ArgumentException(
                "NaN in lookAtTargetReward.\n" +
                $" cubeForward: {cubeForward}\n" +
                $" head.forward: {head.forward}"
            );
        }*/

        //standTime++;
        //AddReward(standTime * 0.01f);
        //AddReward(matchSpeedReward * lookAtTargetReward);
        
        //AddReward(lookAtTargetReward / 100);

        Vector3 weaponPivotVel = weaponPivot.position - prevWeaponPivotPos;
        float weaponPivotSpeed = weaponPivotVel.magnitude / Time.fixedDeltaTime;
        float matchDirection =
            (Vector3.Dot(target.position - weaponPivot.position, weaponPivot.forward * -1) + 1) * 0.5f;
        prevWeaponPivotPos = weaponPivot.position;
        weaponAimReward = Mathf.Exp(-10 * Vector3.Distance(target.position, weaponPivot.position)) * weaponPivotSpeed * matchDirection;
        
        AddReward(weaponAimReward);
        aimSlider.value = matchDirection;
        aimRewardText.text = weaponAimReward.ToString("F5");

        Vector2 footLCoMProj = new Vector2(
            m_JdController.bodyPartsDict[footL].joint.worldCenterOfMass.x,
            m_JdController.bodyPartsDict[footL].joint.worldCenterOfMass.z);
        Vector2 footRCoMProj = new Vector2(
            m_JdController.bodyPartsDict[footR].joint.worldCenterOfMass.x,
            m_JdController.bodyPartsDict[footR].joint.worldCenterOfMass.z);
        Vector2 hipsCoMProj = new Vector2(
            m_JdController.bodyPartsDict[hips].joint.worldCenterOfMass.x,
            m_JdController.bodyPartsDict[hips].joint.worldCenterOfMass.z);

        float unstableCoMPenalty = Mathf.Exp(-10 * DistanceLineAndPoint(footLCoMProj, footRCoMProj, hipsCoMProj)) - 1;
        AddReward(unstableCoMPenalty / 50);
    }

    public void ContactGround(float penalty)
    {
        AddReward(penalty * 1000);
        //AddReward((float)(MaxStep - StepCount) / MaxStep * 1000 * penalty);
    }
    
    public void SetTorsoMass()
    {
        m_JdController.bodyPartsDict[chest].joint.mass = m_ResetParams.GetWithDefault("chest_mass", 8);
        m_JdController.bodyPartsDict[spine].joint.mass = m_ResetParams.GetWithDefault("spine_mass", 8);
        m_JdController.bodyPartsDict[hips].joint.mass = m_ResetParams.GetWithDefault("hip_mass", 8);
    }

    public void SetResetParameters()
    {
        //SetTorsoMass();
    }

    /*private void OnDrawGizmos()
    {
        Gizmos.DrawLine(weaponPivot.position, weaponPivot.position + weaponPivot.forward * -1);
    }*/

    private float DistanceLineAndPoint(Vector2 ls, Vector2 le, Vector2 p)
    {
        Vector2 sp = p - ls;
        Vector2 se = le - ls;
        Vector2 es = ls - le;
        Vector2 ep = p - le;

        if (DotProduct2D(sp, se) * DotProduct2D(es, ep) >= 0)
        {
            return Mathf.Abs(CrossProduct2D(sp, se) / Vector2.Distance(ls, le));
        }
        else
        {
            return Mathf.Min(Vector2.Distance(ls, p), Vector2.Distance(le, p));
        }
    }
    
    private float DotProduct2D(Vector2 a, Vector2 b)
    {
        return a.x * b.x + a.y + b.y;
    }

    private float CrossProduct2D(Vector2 a, Vector2 b)
    {
        return a.x * b.y - a.y * b.x;
    }
}
