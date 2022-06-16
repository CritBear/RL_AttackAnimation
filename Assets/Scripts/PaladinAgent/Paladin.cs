using System;
using System.Collections;
using System.Collections.Generic;
using PaladinAgent;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.UI;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using UnityEditor;
using PaladinBodyPart = PaladinAgent.BodyPart;

public class Paladin : Agent
{
    [Header("Reward Visualization")] public Slider taskRewardSlider;
    public Slider imitationRewardSlider;
    public Slider orientationSlider;
    public Slider angularVelocitySlider;
    public Slider endEffectorSlider;
    public Slider centerOfmassSlider;
    
    
    private RecordedAnimation[] motions = new RecordedAnimation[2];
    private int currentDataIndex;
    private int currentFrameIndex;

    [Header("Configure")] public Transform target;
    private PaladinTargetController targetController;
    [HideInInspector] public bool hitTarget = false;

    public Transform weapon;
    public Transform weaponHitRangeStart;
    public Transform weaponHitRangeEnd;
    
    [Header("Body Parts")] public Transform hips;
    public Transform leftUpLeg;
    public Transform leftLeg;
    public Transform leftFoot;
    public Transform leftToeBase;
    public Transform rightUpLeg;
    public Transform rightLeg;
    public Transform rightFoot;
    public Transform rightToeBase;
    public Transform spine0;
    public Transform spine1;
    public Transform spine2;
    public Transform leftShoulder;
    public Transform leftArm;
    public Transform leftForeArm;
    public Transform leftHand;
    public Transform neck;
    public Transform head;
    public Transform rightShoulder;
    public Transform rightArm;
    public Transform rightForeArm;
    public Transform rightHand;

    public Transform[] referJoints;

    private PaladinJointController m_JController;

    private Quaternion[] prevReferRotation;

    private void LoadMotionDatas()
    {
        string path = Application.dataPath + "/RecordedAnimations/" + "Slash1.dat";

        FileStream fileStream = new FileStream(path, FileMode.Open);
        BinaryFormatter formatter = new BinaryFormatter();
        motions[0] = formatter.Deserialize(fileStream) as RecordedAnimation;
        fileStream.Close();
        
        path = Application.dataPath + "/RecordedAnimations/" + "Slash2.dat";
        fileStream = new FileStream(path, FileMode.Open);
        motions[1] = formatter.Deserialize(fileStream) as RecordedAnimation;
        fileStream.Close();
    }

    private void Start()
    {
        LoadMotionDatas();

        targetController = target.gameObject.GetComponent<PaladinTargetController>();
        if (targetController)
        {
            targetController.agent = this;
        }

        weapon.gameObject.GetComponent<PaladinGC>().agent = this;
        
        m_JController = GetComponent<PaladinJointController>();
        m_JController.SetupBodyPart(hips);
        m_JController.SetupBodyPart(leftUpLeg);
        m_JController.SetupBodyPart(leftLeg);
        m_JController.SetupBodyPart(leftFoot);
        m_JController.SetupBodyPart(leftToeBase);
        m_JController.SetupBodyPart(rightUpLeg);
        m_JController.SetupBodyPart(rightLeg);
        m_JController.SetupBodyPart(rightFoot);
        m_JController.SetupBodyPart(rightToeBase);
        m_JController.SetupBodyPart(spine0);
        m_JController.SetupBodyPart(spine1);
        m_JController.SetupBodyPart(spine2);
        m_JController.SetupBodyPart(leftShoulder);
        m_JController.SetupBodyPart(leftArm);
        m_JController.SetupBodyPart(leftForeArm);
        m_JController.SetupBodyPart(leftHand);
        m_JController.SetupBodyPart(neck);
        m_JController.SetupBodyPart(head);
        m_JController.SetupBodyPart(rightShoulder);
        m_JController.SetupBodyPart(rightArm);
        m_JController.SetupBodyPart(rightForeArm);
        m_JController.SetupBodyPart(rightHand);

        if (referJoints.Length != m_JController.bpDict.Values.Count)
        {
            Debug.Log("Number of Joints does not matching!");
        }

        prevReferRotation = new Quaternion[referJoints.Length];
    }

    public override void OnEpisodeBegin()
    {
        hitTarget = false;
        float targetAngle = targetController.MoveTargetToRandomPosition();

        /*if (targetAngle > 0)
        {
            currentDataIndex = 1;
        }
        else
        {
            currentDataIndex = 0;
        }*/

        currentDataIndex = 0;

        currentFrameIndex = UnityEngine.Random.Range(0, 10);
        
        UpdateReferMotion(currentDataIndex, currentFrameIndex);
        
        m_JController.bpDict[hips].joint.TeleportRoot(referJoints[0].position, referJoints[0].rotation);
        m_JController.bpDict[hips].joint.velocity = Vector3.zero;
        m_JController.bpDict[hips].joint.angularVelocity = Vector3.zero;
            
        int jointIndex = 0;
            
        foreach (var bp in m_JController.bpDict.Values)
        {
            if (jointIndex == 0)
            {
                jointIndex++;
                continue;
            }

            (Quaternion.Inverse(bp.initialRotation) * referJoints[jointIndex].localRotation).ToAngleAxis(out var angle, out var axis);
            var target = referJoints[jointIndex].localEulerAngles;

            bp.Reset( Mathf.Deg2Rad * angle * axis, Vector3.zero);
            jointIndex++;
        }
        
    }

    public void CollectBodyPartsObservations(PaladinBodyPart bp, VectorSensor sensor)
    {
        sensor.AddObservation(bp.groundContact.touchingGround);
        sensor.AddObservation(bp.jointTransform.localRotation);
        
        if(!bp.joint.isRoot)
        {
            sensor.AddObservation(bp.joint.jointPosition[0]);
            sensor.AddObservation(bp.joint.jointPosition[1]);
            sensor.AddObservation(bp.joint.jointPosition[2]);

            sensor.AddObservation(bp.joint.jointVelocity[0]);
            sensor.AddObservation(bp.joint.jointVelocity[1]);
            sensor.AddObservation(bp.joint.jointVelocity[2]);

            sensor.AddObservation(bp.joint.jointAcceleration[0]);
            sensor.AddObservation(bp.joint.jointAcceleration[1]);
            sensor.AddObservation(bp.joint.jointAcceleration[2]);

            sensor.AddObservation(bp.joint.jointForce[0]);
            sensor.AddObservation(bp.joint.jointForce[1]);
            sensor.AddObservation(bp.joint.jointForce[2]);
        }
        
        sensor.AddObservation(hips.InverseTransformPoint(bp.joint.velocity));
        sensor.AddObservation(hips.InverseTransformPoint(bp.joint.angularVelocity));
    }
    
    public override void CollectObservations(VectorSensor sensor)
    {
        float phase = currentFrameIndex / (float)(motions[currentDataIndex].frameLength - 1);
        sensor.AddObservation(phase);
        
        sensor.AddObservation(hips.InverseTransformPoint(target.position));

        foreach (var bp in m_JController.bpDict.Values)
        {
            CollectBodyPartsObservations(bp, sensor);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        var actions = actionBuffers.ContinuousActions;

        int i = -1;
        foreach (var bp in m_JController.bpDict.Values)
        {
            if (!bp.joint.isRoot)
            {
                bp.SetJointDriveTarget(actions[++i], actions[++i], actions[++i]);
                bp.SetJointStrength(actions[++i], actions[++i], actions[++i]);
            }
        }
    }
    
    private void FixedUpdate()
    {
        currentFrameIndex++;
        if (currentFrameIndex >= motions[currentDataIndex].frameLength)
        {
            EndEpisode();
        }
        else
        {
            UpdateReferMotion(currentDataIndex, currentFrameIndex);
            
            
            /*m_JController.bpDict[hips].joint.TeleportRoot(referJoints[0].position, referJoints[0].rotation);
            m_JController.bpDict[hips].joint.velocity = Vector3.zero;
            m_JController.bpDict[hips].joint.angularVelocity = Vector3.zero;
            
            int jointIndex = 0;
            
            foreach (var bp in m_JController.bpDict.Values)
            {
                if (jointIndex == 0)
                {
                    jointIndex++;
                    continue;
                }

                (Quaternion.Inverse(bp.initialRotation) * referJoints[jointIndex].localRotation).ToAngleAxis(out var angle, out var axis);
                var target = referJoints[jointIndex].localEulerAngles;
                
                Quaternion referDeltaRotation = Quaternion.Inverse(prevReferRotation[jointIndex]) *
                                                referJoints[jointIndex].localRotation;
                referDeltaRotation.ToAngleAxis(out var angle2, out var axis2);
                Vector3 referAngularVelocity = angle2 * Mathf.Deg2Rad / Time.fixedDeltaTime * axis2;

                bp.Reset( Mathf.Deg2Rad * angle * axis, referAngularVelocity);
                jointIndex++;
            }*/
            
            
            // Set Reward
            float imitationReward = GetImitationReward();
            float taskReward = GetTaskReward();

            taskRewardSlider.value = taskReward;
            imitationRewardSlider.value = imitationReward;
            
            AddReward(imitationReward * 0.9f + taskReward * 0.1f);
        }
    }

    private float GetImitationReward()
    {
        float reward = 0;
        
        // Orientation Matching Reward
        float orientationReward = Mathf.Exp(-0.1f * GetOrientationSqrResidual()); // -2
        reward += orientationReward * 0.65f;
        
        // Angular Velocity Matching Reward
        float angularVelocityReward = Mathf.Exp(-0.0005f * GetAngularVelocitySqrResidual()); // -0.1
        reward += angularVelocityReward * 0.1f;
        
        // End Effector Position Matching Reward
        float endEffectorReward = Mathf.Exp(-4 * GetEndEffectorPositionSqrResidual()); // -40
        reward += endEffectorReward * 0.15f;
        
        // Center of Mass Matching Reward
        float centerOfMassReward = Mathf.Exp(-10 * GetCenterOfMassSqrResidual()); // -10
        reward += centerOfMassReward * 0.1f;

        orientationSlider.value = orientationReward;
        angularVelocitySlider.value = angularVelocityReward;
        endEffectorSlider.value = endEffectorReward;
        centerOfmassSlider.value = centerOfMassReward;

        return reward;
    }

    private float GetTaskReward()
    {
        if (hitTarget)
        {
            return 1;
        }

        float distance = HandleUtility.DistancePointLine(
            target.position,
            weaponHitRangeStart.position,
            weaponHitRangeEnd.position
        );

        float reward = Mathf.Exp(-4 * distance * distance);

        return reward;
    }

    private float GetOrientationSqrResidual()
    {
        float totalResidual = 0f;
        int jointIndex = 0;
        foreach (var bp in m_JController.bpDict.Values)
        {
            (
                Quaternion.Inverse(referJoints[jointIndex].localRotation) * bp.jointTransform.localRotation
            ).ToAngleAxis(out var angle, out var axis);

            float rad = angle * Mathf.Deg2Rad;
            totalResidual += rad * rad;
            
            jointIndex++;
        }

        return totalResidual;
    }

    private float GetAngularVelocitySqrResidual()
    {
        float totalResidual = 0f;
        int jointIndex = 0;
        foreach (var bp in m_JController.bpDict.Values)
        {
            Quaternion referDeltaRotation = Quaternion.Inverse(prevReferRotation[jointIndex]) *
                                            referJoints[jointIndex].localRotation;
            referDeltaRotation.ToAngleAxis(out var angle, out var axis);
            Vector3 referAngularVelocity = angle * Mathf.Deg2Rad / Time.fixedDeltaTime * axis;

            totalResidual += (referAngularVelocity - bp.joint.angularVelocity).sqrMagnitude;
            
            jointIndex++;
        }

        return totalResidual;
    }

    private float GetEndEffectorPositionSqrResidual()
    {
        float totalResidual = 0f;
        
        // Joint Index
        // 4  : LeftToeBase
        // 8  : RightToeBase
        // 15 : LeftHand
        // 21 : RightHand

        totalResidual += (referJoints[4].position - leftToeBase.position).sqrMagnitude;
        totalResidual += (referJoints[8].position - rightToeBase.position).sqrMagnitude;
        totalResidual += (referJoints[15].position - leftHand.position).sqrMagnitude;
        totalResidual += (referJoints[21].position - rightHand.position).sqrMagnitude;

        return totalResidual;
    }
    
    private float GetCenterOfMassSqrResidual()
    {
        Vector3 referCenterOfMass = Vector3.zero;
        Vector3 agentCenterOfMass = Vector3.zero;

        int jointIndex = 0;
        float totalMass = 0f;
        
        foreach (var bp in m_JController.bpDict.Values)
        {
            totalMass += bp.mass;
            Vector3 localCenterOfMass = bp.joint.centerOfMass;
            referCenterOfMass += (referJoints[jointIndex].position - localCenterOfMass) * bp.mass;
            agentCenterOfMass += (bp.jointTransform.position - localCenterOfMass) * bp.mass;

            jointIndex++;
        }

        referCenterOfMass /= totalMass;
        agentCenterOfMass /= totalMass;

        float sqrResidual = (agentCenterOfMass - referCenterOfMass).sqrMagnitude;

        return sqrResidual;
    }
    
    private void UpdateReferMotion(int dataIndex, int frameIndex)
    {
        Vector3 referRootPos = new Vector3(
            motions[dataIndex].rootPositionsX[frameIndex],
            motions[dataIndex].rootPositionsY[frameIndex],
            motions[dataIndex].rootPositionsZ[frameIndex]
        );
        referJoints[0].localPosition = referRootPos;

        for (int i = 0; i < referJoints.Length; i++)
        {
            Quaternion referJointRot = new Quaternion(
                motions[dataIndex].jointsRotationsX[frameIndex][i],
                motions[dataIndex].jointsRotationsY[frameIndex][i],
                motions[dataIndex].jointsRotationsZ[frameIndex][i],
                motions[dataIndex].jointsRotationsW[frameIndex][i]
            );

            prevReferRotation[i] = referJoints[i].localRotation;
            referJoints[i].localRotation = referJointRot;
        }
    }
}
